#!/usr/bin/env python3
import math
import time
from copy import deepcopy

import rospy
from gazebo_msgs.msg import ModelState, ModelStates
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    q = deepcopy(PoseStamped().pose.orientation)
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q


class AirMotionExecutor:
    def __init__(self):
        self.model_name = rospy.get_param("~model_name", "uav_v4")
        self.rate_hz = rospy.get_param("~update_rate", 30.0)
        self.cruise_altitude = rospy.get_param("~cruise_altitude", 1.2)
        self.takeoff_speed = rospy.get_param("~takeoff_speed", 0.4)
        self.land_speed = rospy.get_param("~land_speed", 0.35)
        self.air_xy_speed = rospy.get_param("~air_xy_speed", 0.7)
        self.ground_altitude = rospy.get_param("~ground_altitude", 0.0)
        self.heading_offset = math.radians(rospy.get_param("~heading_offset_deg", 0.0))
        goal_topic = rospy.get_param("~goal_topic", "/rescue/air_goal")

        self.mode = "GROUND"
        self.goal = None
        self.odom = None
        self.model_pose = None
        self.last_time = time.monotonic()

        self.air_cmd_pub = rospy.Publisher("/air_stub/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/rescue/mode", String, self.mode_cb, queue_size=1)
        rospy.Subscriber(goal_topic, PoseStamped, self.goal_cb, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_cb, queue_size=1)

        rospy.loginfo("Waiting for /gazebo/set_model_state ...")
        rospy.wait_for_service("/gazebo/set_model_state")
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    def mode_cb(self, msg):
        self.mode = msg.data

    def goal_cb(self, msg):
        self.goal = msg

    def odom_cb(self, msg):
        self.odom = msg

    def model_states_cb(self, msg):
        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            return
        self.model_pose = msg.pose[index]

    def step(self):
        cmd = Twist()
        if self.odom is None and self.model_pose is None:
            self.air_cmd_pub.publish(cmd)
            return

        now = time.monotonic()
        dt = max(0.0, min(0.1, now - self.last_time))
        self.last_time = now

        pose = self.model_pose if self.model_pose is not None else self.odom.pose.pose
        x = pose.position.x
        y = pose.position.y
        z = pose.position.z
        next_x = x
        next_y = y
        next_z = z
        desired_yaw = yaw_from_quaternion(pose.orientation)

        if self.mode == "TAKEOFF":
            cmd.linear.z = self.takeoff_speed
            next_z = min(self.cruise_altitude, z + self.takeoff_speed * dt)
        elif self.mode == "AIR":
            next_z = self.cruise_altitude
            if self.goal is not None:
                dx = self.goal.pose.position.x - x
                dy = self.goal.pose.position.y - y
                dist = math.hypot(dx, dy)
                if dist > 1e-3:
                    step = min(self.air_xy_speed * dt, dist)
                    next_x = x + dx / dist * step
                    next_y = y + dy / dist * step
                    cmd.linear.x = dx / dist * self.air_xy_speed
                    cmd.linear.y = dy / dist * self.air_xy_speed
                    desired_yaw = math.atan2(dy, dx)
        elif self.mode == "LAND":
            cmd.linear.z = -self.land_speed
            next_z = max(self.ground_altitude, z - self.land_speed * dt)
        else:
            self.air_cmd_pub.publish(cmd)
            return

        if self.mode in ("TAKEOFF", "LAND") and self.goal is not None:
            dx = self.goal.pose.position.x - x
            dy = self.goal.pose.position.y - y
            if math.hypot(dx, dy) > 0.1:
                desired_yaw = math.atan2(dy, dx)

        state = ModelState()
        state.model_name = self.model_name
        state.reference_frame = "world"
        state.pose = deepcopy(pose)
        state.pose.position.x = next_x
        state.pose.position.y = next_y
        state.pose.position.z = next_z
        state.pose.orientation = quaternion_from_yaw(desired_yaw + self.heading_offset)
        state.twist = cmd

        try:
            self.set_model_state(state)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "set_model_state failed: %s", exc)

        self.air_cmd_pub.publish(cmd)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("air_motion_executor")
    AirMotionExecutor().spin()
