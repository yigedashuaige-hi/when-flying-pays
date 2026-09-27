#!/usr/bin/env python3
import math
import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String
from tf.transformations import euler_from_quaternion

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

import terrain_field


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wrap_pi(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class GroundGoalController:
    def __init__(self):
        self.rate_hz = rospy.get_param("~update_rate", 20.0)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.25)
        self.max_linear_speed = rospy.get_param("~max_linear_speed", 0.45)
        self.max_angular_speed = rospy.get_param("~max_angular_speed", 1.2)
        self.linear_gain = rospy.get_param("~linear_gain", 0.7)
        self.angular_gain = rospy.get_param("~angular_gain", 1.8)
        self.recovery_goal_tolerance = rospy.get_param(
            "~recovery_goal_tolerance", 0.10)
        self.recovery_yaw_tolerance = rospy.get_param(
            "~recovery_yaw_tolerance", 0.10)
        self.terrain_effects_enabled = rospy.get_param("~terrain_effects_enabled", False)
        self.min_traction = rospy.get_param("~min_traction", 0.20)
        self.stuck_traction = rospy.get_param("~stuck_traction", 0.0)
        self.terrain_cost_gain = rospy.get_param("~terrain_cost_gain", 0.70)
        self.obstacle_density_gain = rospy.get_param("~obstacle_density_gain", 0.35)
        self.stuck_risk_gain = rospy.get_param("~stuck_risk_gain", 0.45)
        self.stuck_risk_threshold = rospy.get_param("~stuck_risk_threshold", 0.72)
        self.stuck_obstacle_threshold = rospy.get_param("~stuck_obstacle_threshold", 0.80)
        # Seedable stuck/slip physics (paper-hardening). seed < 0 keeps the legacy
        # deterministic terrain degradation below; seed >= 0 draws a per-cell
        # Bernoulli trap governed by ground-truth traversability (theta_true).
        self.seed = int(rospy.get_param("~seed", -1))
        self.cell_size = float(rospy.get_param("~cell_size", 0.5))
        # Stuck model (explicit research axis): "permanent" = once a risky cell
        # fires Bernoulli(1-theta), traction collapses to stuck_traction (a hard
        # trap); "recoverable" = continuous slip with theta down to min_traction,
        # so the robot always crawls out (more realistic). Reported side by side.
        self.stuck_model = str(rospy.get_param("~stuck_model", "permanent"))
        command_topic = rospy.get_param("~command_topic", "/mecanum/cmd_vel")
        nominal_command_topic = rospy.get_param("~nominal_command_topic", "/rescue/ground_cmd_nominal")

        self.mode = "GROUND"
        self.odom = None
        self.goal = None
        self.recovery_goal = None
        self.recovery_active = False
        self.terrain_cost = 0.0
        self.obstacle_density = 0.0
        self.stuck_risk_prior = 0.0
        self.traversability_true = 1.0

        self.cmd_pub = rospy.Publisher(command_topic, Twist, queue_size=10)
        self.nominal_cmd_pub = rospy.Publisher(nominal_command_topic, Twist, queue_size=10)
        rospy.Subscriber("/rescue/mode", String, self.mode_cb, queue_size=1)
        rospy.Subscriber("/rescue/goal", PoseStamped, self.goal_cb, queue_size=1)
        rospy.Subscriber(
            "/rescue/rotors/reposition_goal", PoseStamped,
            self.recovery_goal_cb, queue_size=1)
        rospy.Subscriber(
            "/rescue/rotors/reposition_active", Bool,
            self.recovery_active_cb, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/rescue/terrain_cost", Float32, self.terrain_cost_cb, queue_size=1)
        rospy.Subscriber("/rescue/obstacle_density", Float32, self.obstacle_density_cb, queue_size=1)
        rospy.Subscriber("/rescue/stuck_risk_prior", Float32, self.stuck_risk_prior_cb, queue_size=1)
        rospy.Subscriber("/rescue/traversability_true", Float32, self.traversability_true_cb, queue_size=1)

    def mode_cb(self, msg):
        self.mode = msg.data

    def goal_cb(self, msg):
        self.goal = msg

    def recovery_goal_cb(self, msg):
        self.recovery_goal = msg

    def recovery_active_cb(self, msg):
        self.recovery_active = bool(msg.data)

    def odom_cb(self, msg):
        self.odom = msg

    def terrain_cost_cb(self, msg):
        self.terrain_cost = clamp(msg.data, 0.0, 1.0)

    def obstacle_density_cb(self, msg):
        self.obstacle_density = clamp(msg.data, 0.0, 1.0)

    def stuck_risk_prior_cb(self, msg):
        self.stuck_risk_prior = clamp(msg.data, 0.0, 1.0)

    def traversability_true_cb(self, msg):
        self.traversability_true = clamp(msg.data, 0.0, 1.0)

    def yaw_from_odom(self):
        q = self.odom.pose.pose.orientation
        return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def compute_cmd(self):
        cmd = Twist()
        goal = self.recovery_goal if self.recovery_active else self.goal
        if self.mode != "GROUND" or self.odom is None or goal is None:
            return cmd

        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        gx = goal.pose.position.x
        gy = goal.pose.position.y
        dx = gx - px
        dy = gy - py
        distance = math.hypot(dx, dy)
        tolerance = (self.recovery_goal_tolerance
                     if self.recovery_active else self.goal_tolerance)
        if distance < tolerance:
            if self.recovery_active:
                q = goal.pose.orientation
                goal_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                yaw_error = wrap_pi(goal_yaw - self.yaw_from_odom())
                if abs(yaw_error) >= self.recovery_yaw_tolerance:
                    cmd.angular.z = clamp(
                        self.angular_gain * yaw_error,
                        -self.max_angular_speed, self.max_angular_speed)
            return cmd

        yaw = self.yaw_from_odom()
        target_yaw = math.atan2(dy, dx)
        yaw_error = wrap_pi(target_yaw - yaw)

        speed = clamp(self.linear_gain * distance, 0.0, self.max_linear_speed)
        cmd.linear.x = speed * math.cos(yaw_error)
        cmd.linear.y = speed * math.sin(yaw_error)
        cmd.angular.z = clamp(self.angular_gain * yaw_error, -self.max_angular_speed, self.max_angular_speed)
        return cmd

    def traction_factor(self):
        """Traction multiplier in [stuck_traction, 1].

        seed >= 0: governed by ground-truth traversability theta_true — a
        continuous slip plus a per-cell Bernoulli trap (seed-keyed, so the same
        cell traps identically for every strategy). seed < 0: legacy deterministic
        degradation from the perceived terrain quantities.
        """
        if self.seed >= 0:
            theta = self.traversability_true
            traction = clamp(theta, self.min_traction, 1.0)
            # Recoverable model: continuous slip only (no hard trap) -> the robot
            # always crawls out at >= min_traction speed.
            if self.stuck_model == "recoverable":
                return traction
            # Permanent model: a fired risky cell is a hard trap.
            if self.odom is not None:
                x = self.odom.pose.pose.position.x
                y = self.odom.pose.pose.position.y
                if terrain_field.stuck_draw(self.seed, x, y, theta, self.cell_size):
                    traction = self.stuck_traction
            return traction

        traction = 1.0
        traction -= self.terrain_cost_gain * self.terrain_cost
        traction -= self.obstacle_density_gain * self.obstacle_density
        traction -= self.stuck_risk_gain * self.stuck_risk_prior
        traction = clamp(traction, self.min_traction, 1.0)
        if (
            self.stuck_risk_prior >= self.stuck_risk_threshold
            or self.obstacle_density >= self.stuck_obstacle_threshold
        ):
            traction = self.stuck_traction
        return traction

    def apply_terrain_effects(self, cmd):
        if not self.terrain_effects_enabled or self.mode != "GROUND":
            return cmd

        traction = self.traction_factor()

        degraded = Twist()
        degraded.linear.x = cmd.linear.x * traction
        degraded.linear.y = cmd.linear.y * traction
        degraded.linear.z = cmd.linear.z * traction
        degraded.angular.x = cmd.angular.x
        degraded.angular.y = cmd.angular.y
        degraded.angular.z = cmd.angular.z * traction
        return degraded

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            nominal_cmd = self.compute_cmd()
            self.nominal_cmd_pub.publish(nominal_cmd)
            self.cmd_pub.publish(self.apply_terrain_effects(nominal_cmd))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ground_goal_controller")
    GroundGoalController().spin()
