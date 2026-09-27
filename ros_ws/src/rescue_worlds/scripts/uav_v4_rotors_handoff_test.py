#!/usr/bin/env python3
"""Phase E1a-H isolated, measured-feedback handoff validation harness."""

import argparse
import csv
import json
import math
import os

import rospy
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, PoseStamped, Twist, Vector3Stamped, WrenchStamped
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int32, String
from tf.transformations import euler_from_quaternion


def mean_abs(values):
    return sum(abs(value) for value in values) / len(values) if values else 0.0


class HandoffTest:
    def __init__(self, scenario, timeline_path):
        self.scenario = scenario
        self.timeline_path = timeline_path
        self.odom = None
        self.requested = "GROUND"
        self.approved = ""
        self.handoff = ""
        self.flight_state = ""
        self.armed = False
        self.disarmed = True
        self.ground_active = False
        self.owner_command = True
        self.motor_enable = False
        self.contact = False
        self.ground_force = [0.0] * 3
        self.ground_torque = [0.0] * 3
        self.command_pose = None
        self.raw_motor = []
        self.gated_motor = []
        self.actual_motor = []
        self.nonfinite = []
        self.dual_ownership_samples = 0
        self.state_edges = []
        self.max_takeoff_roll = 0.0
        self.max_takeoff_pitch = 0.0
        self.vertical_xy_origin = None
        self.max_vertical_xy_drift = 0.0
        self.first_horizontal_height = None
        self.stages = []
        self.site_valid = False
        self.site_measurement_valid = False
        self.site_available = float("nan")
        self.site_required = float("nan")
        self.site_nearest = float("nan")
        self.site_reason = ""
        self.site_recheck_distance = float("nan")
        self.obstacle_contact = False
        self.obstacle_contact_ever = False
        self.obstacle_contact_force = 0.0
        self.max_obstacle_contact_force = 0.0
        self.takeoff_decision_reason = ""
        self.blocked_duration = 0.0
        self.blocked_distance = 0.0
        self.approved_takeoff_pose = None
        self.special_acceptance = None
        self.recovery_scenario = scenario.startswith("reposition_")
        self.reposition_active = False
        self.reposition_ever = False
        self.reposition_goal = None
        self.reposition_duration = 0.0
        self.reposition_distance = 0.0
        self.reposition_initial_distance = 0.0
        self.reposition_history_age = 0.0
        self.reposition_result = "none"
        self.persistent_site_count = 0
        self.max_persistent_site_count = 0
        self.reposition_target_site_id = 0
        self.reposition_target_source = "none"
        self.reposition_target_reachability = "none"
        self.persistent_cross_sortie_candidate_seen = False
        self.reposition_target_revalidated = False
        self.max_reposition_motor_mean_abs = 0.0
        self.invalid_takeoff_motor_samples = 0

        self.request_pub = rospy.Publisher(
            "/rescue/requested_mode", String, queue_size=1, latch=True
        )
        self.goal_pub = rospy.Publisher("/rescue/goal", PoseStamped, queue_size=1, latch=True)
        self.ground_pub = rospy.Publisher("/mecanum/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("/rescue/mode", String, self.approved_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/handoff_state", String, self.handoff_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/flight_state", String, self.flight_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/armed", Bool, self.armed_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/disarmed", Bool, self.disarmed_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/ground_ownership", Bool, self.owner_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/motor_enable", Bool, self.motor_enable_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/contact_inferred", Bool, self.contact_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/ground_control_active", Bool, self.active_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/ground_control_wrench", WrenchStamped,
                         self.wrench_cb, queue_size=1)
        rospy.Subscriber("/uav_v4/command/pose", PoseStamped, self.command_cb, queue_size=1)
        rospy.Subscriber("/uav_v4/odometry", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/uav_v4/command/motor_speed_raw", Actuators,
                         lambda msg: self.motor_cb("raw_motor", msg), queue_size=1)
        rospy.Subscriber("/uav_v4/command/motor_speed", Actuators,
                         lambda msg: self.motor_cb("gated_motor", msg), queue_size=1)
        rospy.Subscriber("/uav_v4/motor_speed", Actuators,
                         lambda msg: self.motor_cb("actual_motor", msg), queue_size=1)
        rospy.Subscriber("/rescue/rotors/takeoff_clearance", Vector3Stamped,
                         self.site_status_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/takeoff_site_valid", Bool,
                         lambda msg: setattr(self, "site_valid", bool(msg.data)), queue_size=1)
        rospy.Subscriber("/rescue/rotors/takeoff_measurement_valid", Bool,
                         lambda msg: setattr(self, "site_measurement_valid", bool(msg.data)), queue_size=1)
        rospy.Subscriber("/rescue/rotors/takeoff_blocked_reason", String,
                         lambda msg: setattr(self, "site_reason", msg.data), queue_size=1)
        rospy.Subscriber("/rescue/rotors/takeoff_recheck_distance", Float32,
                         lambda msg: setattr(self, "site_recheck_distance", msg.data), queue_size=1)
        rospy.Subscriber("/rescue/rotors/obstacle_contact", Bool,
                         self.obstacle_contact_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/obstacle_contact_force_n", Float32,
                         self.obstacle_contact_force_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/takeoff_decision_reason", String,
                         lambda msg: setattr(self, "takeoff_decision_reason", msg.data), queue_size=1)
        rospy.Subscriber("/rescue/rotors/takeoff_blocked_duration_s", Float32,
                         lambda msg: setattr(self, "blocked_duration", msg.data), queue_size=1)
        rospy.Subscriber("/rescue/rotors/takeoff_blocked_distance_m", Float32,
                         lambda msg: setattr(self, "blocked_distance", msg.data), queue_size=1)
        rospy.Subscriber("/rescue/rotors/approved_takeoff_pose", PoseStamped,
                         lambda msg: setattr(self, "approved_takeoff_pose", msg), queue_size=1)
        rospy.Subscriber("/rescue/rotors/reposition_active", Bool,
                         self.reposition_active_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/reposition_goal", PoseStamped,
                         lambda msg: setattr(self, "reposition_goal", msg), queue_size=1)
        rospy.Subscriber("/rescue/rotors/reposition_duration_s", Float32,
                         lambda msg: setattr(self, "reposition_duration", msg.data), queue_size=1)
        rospy.Subscriber("/rescue/rotors/reposition_distance_m", Float32,
                         lambda msg: setattr(self, "reposition_distance", msg.data), queue_size=1)
        rospy.Subscriber("/rescue/rotors/reposition_initial_distance_m", Float32,
                         lambda msg: setattr(self, "reposition_initial_distance", msg.data), queue_size=1)
        rospy.Subscriber("/rescue/rotors/reposition_target_history_age_s", Float32,
                         lambda msg: setattr(self, "reposition_history_age", msg.data), queue_size=1)
        rospy.Subscriber("/rescue/rotors/reposition_result", String,
                         lambda msg: setattr(self, "reposition_result", msg.data), queue_size=1)
        rospy.Subscriber("/rescue/rotors/persistent_safe_site_count", Int32,
                         self.persistent_site_count_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/reposition_target_site_id", Int32,
                         lambda msg: setattr(self, "reposition_target_site_id", msg.data),
                         queue_size=1)
        rospy.Subscriber("/rescue/rotors/reposition_target_source", String,
                         lambda msg: setattr(self, "reposition_target_source", msg.data),
                         queue_size=1)
        rospy.Subscriber("/rescue/rotors/reposition_target_reachability", String,
                         self.reposition_target_reachability_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/reposition_target_revalidated", Bool,
                         lambda msg: setattr(self, "reposition_target_revalidated",
                                             bool(msg.data)), queue_size=1)

        os.makedirs(os.path.dirname(os.path.abspath(timeline_path)), exist_ok=True)
        self.stream = open(timeline_path, "w", newline="")
        self.fields = [
            "sim_time_s", "requested_mode", "approved_mode", "handoff_state",
            "flight_state", "armed", "disarmed", "ground_owner_command",
            "ground_active", "motor_enable", "contact_inferred", "cmd_x", "cmd_y",
            "cmd_yaw", "force_x_n", "force_y_n", "force_z_n", "torque_x_nm",
            "torque_y_nm", "torque_z_nm", "command_x", "command_y", "command_z",
            "x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw",
            "raw_motor_mean_abs", "gated_motor_mean_abs", "actual_motor_mean_abs",
            "site_measurement_valid", "site_valid", "site_available_m",
            "site_required_m", "site_nearest_m", "site_reason",
            "site_recheck_distance_m", "obstacle_contact",
            "obstacle_contact_force_n", "takeoff_decision_reason",
            "blocked_duration_s", "blocked_distance_m",
            "reposition_active", "reposition_goal_x", "reposition_goal_y",
            "reposition_duration_s", "reposition_distance_m",
            "reposition_initial_distance_m", "reposition_history_age_s",
            "reposition_result",
            "persistent_site_count", "reposition_target_site_id",
            "reposition_target_source", "reposition_target_reachability",
            "reposition_target_revalidated",
        ]
        self.writer = csv.DictWriter(self.stream, fieldnames=self.fields)
        self.writer.writeheader()

    def approved_cb(self, msg): self.approved = msg.data
    def flight_cb(self, msg): self.flight_state = msg.data
    def armed_cb(self, msg): self.armed = bool(msg.data)
    def disarmed_cb(self, msg): self.disarmed = bool(msg.data)
    def owner_cb(self, msg): self.owner_command = bool(msg.data)
    def motor_enable_cb(self, msg): self.motor_enable = bool(msg.data)
    def contact_cb(self, msg): self.contact = bool(msg.data)
    def active_cb(self, msg): self.ground_active = bool(msg.data)

    def reposition_active_cb(self, msg):
        self.reposition_active = bool(msg.data)
        self.reposition_ever = self.reposition_ever or self.reposition_active

    def persistent_site_count_cb(self, msg):
        self.persistent_site_count = int(msg.data)
        self.max_persistent_site_count = max(
            self.max_persistent_site_count, self.persistent_site_count)

    def reposition_target_reachability_cb(self, msg):
        self.reposition_target_reachability = msg.data
        self.persistent_cross_sortie_candidate_seen = (
            self.persistent_cross_sortie_candidate_seen
            or msg.data == "unverified_cross_sortie"
        )

    def handoff_cb(self, msg):
        if msg.data != self.handoff:
            self.handoff = msg.data
            self.state_edges.append([rospy.Time.now().to_sec(), msg.data])
            if msg.data == "TAKEOFF_VERTICAL" and self.odom is not None:
                p = self.odom.pose.pose.position
                self.vertical_xy_origin = (p.x, p.y)
            if msg.data == "AIR_HORIZONTAL_TRACK" and self.odom is not None:
                self.first_horizontal_height = self.odom.pose.pose.position.z

    def wrench_cb(self, msg):
        f, t = msg.wrench.force, msg.wrench.torque
        self.ground_force = [f.x, f.y, f.z]
        self.ground_torque = [t.x, t.y, t.z]

    def command_cb(self, msg): self.command_pose = msg

    def site_status_cb(self, msg):
        self.site_available = msg.vector.x
        self.site_required = msg.vector.y
        self.site_nearest = msg.vector.z

    def obstacle_contact_cb(self, msg):
        self.obstacle_contact = bool(msg.data)
        self.obstacle_contact_ever = self.obstacle_contact_ever or self.obstacle_contact

    def obstacle_contact_force_cb(self, msg):
        self.obstacle_contact_force = msg.data
        self.max_obstacle_contact_force = max(
            self.max_obstacle_contact_force, msg.data
        )

    def odom_cb(self, msg):
        self.odom = msg
        if not all(math.isfinite(value) for value in self.odom_values()):
            self.nonfinite.append([rospy.Time.now().to_sec(), "odometry"])

    def motor_cb(self, name, msg):
        values = list(msg.angular_velocities)
        setattr(self, name, values)
        if not all(math.isfinite(value) for value in values):
            self.nonfinite.append([rospy.Time.now().to_sec(), name])

    def odom_values(self):
        if self.odom is None:
            return []
        p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
        v, w = self.odom.twist.twist.linear, self.odom.twist.twist.angular
        return [p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z, w.x, w.y, w.z]

    def pose(self):
        p, q = self.odom.pose.pose.position, self.odom.pose.pose.orientation
        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return p.x, p.y, p.z, roll, pitch, yaw

    def hspeed(self):
        v = self.odom.twist.twist.linear
        return math.hypot(v.x, v.y)

    def publish(self, requested, goal, ground_cmd=None):
        self.requested = requested
        self.request_pub.publish(String(data=requested))
        goal.header.stamp = rospy.Time.now()
        self.goal_pub.publish(goal)
        if not self.recovery_scenario:
            self.ground_pub.publish(ground_cmd if ground_cmd is not None else Twist())

    @staticmethod
    def make_goal(x, y):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.pose.position.x, msg.pose.position.y = x, y
        msg.pose.orientation.w = 1.0
        return msg

    def sample(self, ground_cmd):
        if self.odom is None:
            return
        x, y, z, roll, pitch, yaw = self.pose()
        v = self.odom.twist.twist.linear
        if self.handoff in ("TAKEOFF_VERTICAL", "TAKEOFF_CLEARANCE_CONFIRM"):
            self.max_takeoff_roll = max(self.max_takeoff_roll, abs(roll))
            self.max_takeoff_pitch = max(self.max_takeoff_pitch, abs(pitch))
            if self.vertical_xy_origin is not None:
                self.max_vertical_xy_drift = max(
                    self.max_vertical_xy_drift,
                    math.hypot(x - self.vertical_xy_origin[0], y - self.vertical_xy_origin[1]),
                )
        if self.ground_active and (mean_abs(self.gated_motor) > 5.0
                                   or mean_abs(self.actual_motor) > 5.0):
            self.dual_ownership_samples += 1
        if self.reposition_active:
            self.max_reposition_motor_mean_abs = max(
                self.max_reposition_motor_mean_abs,
                mean_abs(self.actual_motor),
            )
        if (self.handoff in ("TAKEOFF_VERTICAL", "TAKEOFF_CLEARANCE_CONFIRM")
                and self.motor_enable and not self.site_valid):
            self.invalid_takeoff_motor_samples += 1
        command = self.command_pose.pose.position if self.command_pose else None
        reposition = self.reposition_goal.pose.position if self.reposition_goal else None
        self.writer.writerow({
            "sim_time_s": rospy.Time.now().to_sec(), "requested_mode": self.requested,
            "approved_mode": self.approved, "handoff_state": self.handoff,
            "flight_state": self.flight_state, "armed": int(self.armed),
            "disarmed": int(self.disarmed), "ground_owner_command": int(self.owner_command),
            "ground_active": int(self.ground_active), "motor_enable": int(self.motor_enable),
            "contact_inferred": int(self.contact), "cmd_x": ground_cmd.linear.x,
            "cmd_y": ground_cmd.linear.y, "cmd_yaw": ground_cmd.angular.z,
            "force_x_n": self.ground_force[0], "force_y_n": self.ground_force[1],
            "force_z_n": self.ground_force[2], "torque_x_nm": self.ground_torque[0],
            "torque_y_nm": self.ground_torque[1], "torque_z_nm": self.ground_torque[2],
            "command_x": command.x if command else "", "command_y": command.y if command else "",
            "command_z": command.z if command else "", "x": x, "y": y, "z": z,
            "vx": v.x, "vy": v.y, "vz": v.z, "roll": roll, "pitch": pitch, "yaw": yaw,
            "raw_motor_mean_abs": mean_abs(self.raw_motor),
            "gated_motor_mean_abs": mean_abs(self.gated_motor),
            "actual_motor_mean_abs": mean_abs(self.actual_motor),
            "site_measurement_valid": int(self.site_measurement_valid),
            "site_valid": int(self.site_valid),
            "site_available_m": self.site_available,
            "site_required_m": self.site_required,
            "site_nearest_m": self.site_nearest,
            "site_reason": self.site_reason,
            "site_recheck_distance_m": self.site_recheck_distance,
            "obstacle_contact": int(self.obstacle_contact),
            "obstacle_contact_force_n": self.obstacle_contact_force,
            "takeoff_decision_reason": self.takeoff_decision_reason,
            "blocked_duration_s": self.blocked_duration,
            "blocked_distance_m": self.blocked_distance,
            "reposition_active": int(self.reposition_active),
            "reposition_goal_x": reposition.x if reposition else "",
            "reposition_goal_y": reposition.y if reposition else "",
            "reposition_duration_s": self.reposition_duration,
            "reposition_distance_m": self.reposition_distance,
            "reposition_initial_distance_m": self.reposition_initial_distance,
            "reposition_history_age_s": self.reposition_history_age,
            "reposition_result": self.reposition_result,
            "persistent_site_count": self.persistent_site_count,
            "reposition_target_site_id": self.reposition_target_site_id,
            "reposition_target_source": self.reposition_target_source,
            "reposition_target_reachability": self.reposition_target_reachability,
            "reposition_target_revalidated": int(self.reposition_target_revalidated),
        })

    def wait(self, name, condition, timeout, requested, goal, ground_cmd=None, hold=0.0):
        rate = rospy.Rate(50)
        start = rospy.Time.now()
        stable_since = None
        passed = False
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < timeout:
            self.publish(requested, goal, ground_cmd)
            self.sample(ground_cmd if ground_cmd is not None else Twist())
            if self.nonfinite:
                break
            if condition():
                stable_since = stable_since or rospy.Time.now()
                if (rospy.Time.now() - stable_since).to_sec() >= hold:
                    passed = True
                    break
            else:
                stable_since = None
            rate.sleep()
        self.stages.append({"name": name, "passed": passed,
                            "elapsed_sim_s": (rospy.Time.now() - start).to_sec()})
        return passed

    @staticmethod
    def obstacle_sdf(name, size, pose):
        return """<sdf version='1.6'><model name='{name}'><static>true</static>
<link name='link'><collision name='collision'>
<geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>
<visual name='visual'><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
</visual></link></model></sdf>""".format(
            name=name, sx=size[0], sy=size[1], sz=size[2]
        )

    def spawn_obstacle(self, name, size, pose):
        rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=5.0)
        spawn = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
        initial_pose = Pose()
        initial_pose.position.x, initial_pose.position.y, initial_pose.position.z = pose
        initial_pose.orientation.w = 1.0
        response = spawn(name, self.obstacle_sdf(name, size, pose), "",
                         initial_pose, "world")
        if not response.success:
            raise RuntimeError("spawn obstacle failed: " + response.status_message)

    @staticmethod
    def delete_obstacle(name):
        rospy.wait_for_service("/gazebo/delete_model", timeout=5.0)
        delete = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)
        delete(name)

    def wait_ready(self, goal):
        return self.wait(
            "ready",
            lambda: (self.odom is not None and self.handoff == "GROUND_DRIVE"
                     and self.disarmed and mean_abs(self.actual_motor) < 5.0
                     and self.site_measurement_valid),
            20.0, "GROUND", goal, hold=0.3,
        )

    def takeoff_hover_land(self, goal, expect_contact_free=True):
        if not self.wait(
                "takeoff_vertical",
                lambda: self.handoff in ("TAKEOFF_VERTICAL",
                                         "TAKEOFF_CLEARANCE_CONFIRM",
                                         "AIR_HORIZONTAL_TRACK"),
                8.0, "TAKEOFF", goal):
            return False
        if not self.wait(
                "target_height_before_horizontal",
                lambda: (self.handoff == "AIR_HORIZONTAL_TRACK"
                         and self.pose()[2] >= 1.10 and abs(self.odom.twist.twist.linear.z) <= 0.20),
                15.0, "TAKEOFF", goal, hold=0.3):
            return False
        if expect_contact_free and self.obstacle_contact:
            return False
        return self.wait(
            "land_disarm_reacquire",
            lambda: (self.handoff == "GROUND_DRIVE" and self.disarmed
                     and mean_abs(self.actual_motor) < 5.0 and self.contact),
            15.0, "LAND", goal, hold=0.20,
        )

    def drive_with_ground_controller(self, x, timeout=8.0):
        goal = self.make_goal(x, 0.0)
        return self.wait(
            "build_valid_ground_history",
            lambda: self.pose()[0] >= x - 0.25 and self.hspeed() < 0.08,
            timeout, "GROUND", goal, hold=0.25,
        )

    def run_reposition_case(self):
        origin = self.make_goal(0.0, 0.0)
        if not self.wait_ready(origin):
            return False

        if self.scenario == "reposition_current_valid":
            passed = self.takeoff_hover_land(origin)
            self.special_acceptance = {
                "normal_takeoff": passed,
                "reposition_not_triggered": not self.reposition_ever,
                "target_height_before_horizontal": self.first_horizontal_height is not None
                                                   and self.first_horizontal_height >= 1.10,
                "final_ground_disarmed": self.handoff == "GROUND_DRIVE" and self.disarmed,
            }
            return all(self.special_acceptance.values())

        if self.scenario == "reposition_persistent_cross_sortie":
            if not self.drive_with_ground_controller(0.80):
                return False
            current_x = self.pose()[0]
            launch_site = self.make_goal(current_x, 0.0)
            if not self.wait("persistent_site_recorded",
                             lambda: self.persistent_site_count > 0,
                             3.0, "GROUND", launch_site, hold=0.20):
                return False

            flight_goal = self.make_goal(current_x + 0.70, 0.0)
            if not self.wait(
                    "first_sortie_horizontal",
                    lambda: (self.handoff == "AIR_HORIZONTAL_TRACK"
                             and self.pose()[0] >= current_x + 0.45),
                    18.0, "AIR", flight_goal, hold=0.20):
                return False
            if not self.wait(
                    "first_sortie_land_disarm",
                    lambda: (self.handoff == "GROUND_DRIVE" and self.disarmed
                             and mean_abs(self.actual_motor) < 5.0 and self.contact),
                    15.0, "LAND", flight_goal, hold=0.20):
                return False
            memory_after_flight = self.persistent_site_count

            # Let the 12-s reverse-path deque expire so this fixture exercises
            # the independent cross-sortie registry rather than recent history.
            expiry_start = rospy.Time.now()
            if not self.wait(
                    "expire_recent_reverse_history",
                    lambda: (rospy.Time.now() - expiry_start).to_sec() >= 12.20,
                    12.80, "GROUND", flight_goal):
                return False

            landed_x, landed_y = self.pose()[0], self.pose()[1]
            obstacle_name = "persistent_cross_sortie_obstacle"
            self.spawn_obstacle(obstacle_name, (0.10, 0.60, 0.70),
                                (landed_x + 0.235, landed_y, 0.35))
            landed_goal = self.make_goal(landed_x, landed_y)
            if not self.wait("postflight_site_invalid", lambda: not self.site_valid,
                             3.0, "GROUND", landed_goal, hold=0.20):
                return False

            if not self.wait(
                    "persistent_reposition_then_takeoff",
                    lambda: self.handoff in (
                        "TAKEOFF_VERTICAL", "TAKEOFF_CLEARANCE_CONFIRM",
                        "AIR_HORIZONTAL_TRACK"),
                    12.0, "TAKEOFF", landed_goal):
                return False
            if not self.takeoff_hover_land(landed_goal):
                return False
            horizontal_entries = sum(
                1 for _, state in self.state_edges
                if state == "AIR_HORIZONTAL_TRACK")
            self.special_acceptance = {
                "memory_survived_air_land": memory_after_flight > 0,
                "cross_sortie_candidate_selected": (
                    self.persistent_cross_sortie_candidate_seen
                    and self.reposition_target_site_id > 0
                    and self.reposition_target_source != "recent_reverse_path"),
                "arrival_freshly_revalidated": self.reposition_target_revalidated,
                "two_sorties_completed": horizontal_entries >= 2,
                "motor_zero_during_reposition": (
                    self.max_reposition_motor_mean_abs < 5.0),
                "no_dual_ownership": self.dual_ownership_samples == 0,
                "no_invalid_site_takeoff_motor": (
                    self.invalid_takeoff_motor_samples == 0),
                "final_ground_disarmed": (
                    self.handoff == "GROUND_DRIVE" and self.disarmed
                    and mean_abs(self.actual_motor) < 5.0),
            }
            return all(self.special_acceptance.values())

        obstacle_name = "reposition_test_obstacle"
        if self.scenario == "reposition_no_history":
            self.spawn_obstacle(obstacle_name, (0.10, 0.60, 0.70),
                                (0.235, 0.0, 0.35))
            if not self.wait("site_invalid", lambda: not self.site_valid,
                             3.0, "GROUND", origin, hold=0.20):
                return False
            failed = self.wait(
                "no_history_fails_without_arm",
                lambda: (self.handoff == "TAKEOFF_REPOSITION_FAILED"
                         and self.reposition_result.startswith(
                             "takeoff_reposition_failed:no_stable_valid_history")
                         and self.approved == "GROUND" and self.owner_command
                         and not self.motor_enable and mean_abs(self.actual_motor) < 5.0),
                3.0, "TAKEOFF", origin, hold=0.30,
            )
            released = self.wait(
                "failure_request_withdrawal",
                lambda: self.handoff == "GROUND_DRIVE",
                2.0, "GROUND", origin, hold=0.20,
            )
            self.special_acceptance = {
                "explicit_no_history_failure": failed,
                "no_reposition_deadlock": released,
                "motor_zero": mean_abs(self.actual_motor) < 5.0,
            }
            return all(self.special_acceptance.values())

        if not self.drive_with_ground_controller(0.80):
            return False
        current_x = self.pose()[0]
        stopped = self.make_goal(current_x, 0.0)
        if not self.wait("history_settle", lambda: self.hspeed() < 0.05,
                         2.0, "GROUND", stopped, hold=0.25):
            return False

        if self.scenario == "reposition_unreachable":
            # A thin cross-track wall is both the local invalidity and the
            # physical obstruction to the recorded reverse path. No production
            # parameter is changed for this failure fixture.
            obstacle_x = current_x - 0.235
            self.spawn_obstacle(obstacle_name, (0.10, 2.0, 0.70),
                                (obstacle_x, 0.0, 0.35))
        else:
            obstacle_x = current_x + 0.235
            self.spawn_obstacle(obstacle_name, (0.10, 0.60, 0.70),
                                (obstacle_x, 0.0, 0.35))

        if not self.wait("site_invalid", lambda: not self.site_valid,
                         3.0, "GROUND", stopped, hold=0.20):
            return False

        if self.scenario == "reposition_invalid_after_move":
            # Move a short distance tangentially while still under the invalid
            # footprint; this fixture avoids attempting to drive through the
            # obstacle that caused the invalidity.
            forward = self.make_goal(current_x, 0.50)
            if not self.wait(
                    "move_inside_invalid_region",
                    lambda: self.pose()[1] >= 0.05 and not self.site_valid,
                    2.0, "GROUND", forward):
                return False

        if self.scenario == "reposition_unreachable":
            failed = self.wait(
                "unreachable_exits",
                lambda: (self.handoff == "TAKEOFF_REPOSITION_FAILED"
                         and self.reposition_result.startswith(
                             "takeoff_reposition_failed:")),
                7.0, "TAKEOFF", stopped, hold=0.20,
            )
            motor_safe = mean_abs(self.actual_motor) < 5.0
            released = self.wait("unreachable_request_withdrawal",
                                 lambda: self.handoff == "GROUND_DRIVE",
                                 2.0, "GROUND", stopped, hold=0.20)
            self.special_acceptance = {
                "reposition_attempted": self.reposition_ever,
                "bounded_failure": failed,
                "failure_released": released,
                "motor_zero": motor_safe,
            }
            return all(self.special_acceptance.values())

        repositioned = self.wait(
            "backtrack_then_vertical_takeoff",
            lambda: self.handoff in ("TAKEOFF_VERTICAL",
                                     "TAKEOFF_CLEARANCE_CONFIRM",
                                     "AIR_HORIZONTAL_TRACK"),
            12.0, "TAKEOFF", stopped,
        )
        recovered_valid = self.reposition_result == "takeoff_reposition_succeeded"
        if not repositioned or not self.takeoff_hover_land(stopped):
            return False
        self.special_acceptance = {
            "reposition_triggered": self.reposition_ever,
            "historical_goal_selected": self.reposition_goal is not None,
            "moved_on_ground": self.reposition_distance > 0.0,
            "site_valid_before_existing_handoff": recovered_valid,
            "target_height_before_horizontal": self.first_horizontal_height is not None
                                               and self.first_horizontal_height >= 1.10,
            "final_ground_disarmed": self.handoff == "GROUND_DRIVE" and self.disarmed,
        }
        return all(self.special_acceptance.values())

    def run_clearance_case(self):
        goal = self.make_goal(0.0, 0.0)
        if not self.wait_ready(goal):
            return False
        name = "clearance_test_obstacle"

        if self.scenario == "clearance_far_low":
            self.spawn_obstacle(name, (0.10, 0.50, 0.70), (1.0, 0.0, 0.35))
        elif self.scenario in ("clearance_adjacent_invalid",
                               "clearance_invalid_then_move",
                               "clearance_low_above_minimum"):
            self.spawn_obstacle(name, (0.10, 0.50, 0.70), (0.235, 0.0, 0.35))
        elif self.scenario == "clearance_above_cruise":
            self.spawn_obstacle(name, (0.50, 0.50, 0.20), (0.0, 0.0, 1.35))

        blocked_cases = {
            "clearance_adjacent_invalid", "clearance_low_above_minimum",
            "clearance_above_cruise",
        }
        if self.scenario in blocked_cases:
            if not self.wait("site_invalid", lambda: not self.site_valid, 3.0,
                             "GROUND", goal, hold=0.20):
                return False
            passed = self.wait(
                "blocked_stays_ground",
                lambda: (self.handoff == "GROUND_DRIVE" and self.approved == "GROUND"
                         and self.owner_command and not self.motor_enable
                         and mean_abs(self.actual_motor) < 5.0
                         and self.takeoff_decision_reason.startswith("takeoff_blocked:")),
                3.0, "TAKEOFF", goal, hold=1.0,
            )
            self.special_acceptance = {
                "blocked_stays_ground": passed,
                "no_motor_takeoff_output": mean_abs(self.actual_motor) < 5.0,
                "clear_reason": self.takeoff_decision_reason.startswith("takeoff_blocked:"),
                "fresh_measurement": self.site_measurement_valid,
            }
            self.publish("GROUND", goal, Twist())
            return passed and all(self.special_acceptance.values())

        if self.scenario == "clearance_invalid_then_move":
            if not self.wait("initial_site_invalid", lambda: not self.site_valid, 3.0,
                             "GROUND", goal, hold=0.20):
                return False
            drive = Twist(); drive.linear.x = -0.45
            if not self.wait(
                    "ground_move_until_takeoff_recheck",
                    lambda: self.handoff in ("PRE_TAKEOFF_BRAKE", "RELEASE_AND_ARM",
                                             "TAKEOFF_VERTICAL"),
                    8.0, "TAKEOFF", goal, drive):
                return False
            if not self.takeoff_hover_land(goal):
                return False
            self.special_acceptance = {
                "initially_invalid": True,
                "moved_while_public_ground": self.blocked_distance > 0.0,
                "eventually_approved": self.approved_takeoff_pose is not None,
                "target_height_before_horizontal": self.first_horizontal_height is not None
                                                   and self.first_horizontal_height >= 1.10,
                "final_ground_disarmed": self.handoff == "GROUND_DRIVE" and self.disarmed,
            }
            return all(self.special_acceptance.values())

        if self.scenario == "clearance_contact_stall_abort":
            if not self.wait("takeoff_vertical", lambda: self.handoff == "TAKEOFF_VERTICAL",
                             8.0, "TAKEOFF", goal):
                return False
            if not self.wait("ascending", lambda: self.pose()[2] >= 0.10,
                             4.0, "TAKEOFF", goal):
                return False
            self.spawn_obstacle(name, (0.04, 0.50, 0.30),
                                (self.pose()[0] + 0.18, self.pose()[1], 0.30))
            aborted = self.wait(
                "contact_or_clearance_abort",
                lambda: self.handoff == "LAND_VERTICAL",
                3.0, "TAKEOFF", goal, hold=0.15,
            )
            contacted = self.obstacle_contact_ever
            recovered = self.wait(
                "abort_land_disarm_recover",
                lambda: (self.handoff == "GROUND_DRIVE" and self.disarmed
                         and mean_abs(self.actual_motor) < 5.0),
                15.0, "GROUND", goal, hold=0.20,
            )
            self.special_acceptance = {
                "abort_triggered": aborted,
                "physical_contact_observed": contacted,
                "abort_reason_recorded": self.takeoff_decision_reason.startswith("takeoff_abort:"),
                "landed_disarmed_recovered": recovered,
            }
            return all(self.special_acceptance.values())

        passed = self.takeoff_hover_land(goal)
        if self.scenario == "clearance_landing_handshake":
            required = ["LAND_VERTICAL", "TOUCHDOWN_SETTLE", "DISARM_CONFIRM",
                        "GROUND_REACQUIRE", "GROUND_DRIVE"]
            observed = [state for _, state in self.state_edges]
            cursor = 0
            for state in observed:
                if cursor < len(required) and state == required[cursor]:
                    cursor += 1
            self.special_acceptance = {
                "landing_state_order": cursor == len(required),
                "final_ground_disarmed": self.handoff == "GROUND_DRIVE" and self.disarmed,
                "final_motor_zero": mean_abs(self.actual_motor) < 5.0,
            }
        else:
            self.special_acceptance = {
                "sequence": passed,
                "target_height_before_horizontal": self.first_horizontal_height is not None
                                                   and self.first_horizontal_height >= 1.10,
                "no_obstacle_contact": not self.obstacle_contact_ever,
                "final_ground_disarmed": self.handoff == "GROUND_DRIVE" and self.disarmed,
            }
        return passed and all(self.special_acceptance.values())

    def run(self):
        if self.scenario.startswith("reposition_"):
            return self.run_reposition_case()
        if self.scenario.startswith("clearance_"):
            return self.run_clearance_case()
        zero_goal = self.make_goal(0.0, 0.0)
        if not self.wait("ready", lambda: self.odom is not None and self.handoff == "GROUND_DRIVE"
                         and self.disarmed and mean_abs(self.actual_motor) < 5.0,
                         20.0, "GROUND", zero_goal, hold=0.3):
            return False
        x0, y0, _, _, _, _ = self.pose()
        goal = self.make_goal(x0, y0)

        if self.scenario == "moving":
            drive = Twist(); drive.linear.x = 0.45
            if not self.wait("ground_at_commanded_speed", lambda: self.hspeed() >= 0.40,
                             8.0, "GROUND", goal, drive, hold=0.30):
                return False
        elif self.scenario == "yaw":
            if not self.wait("small_yaw_offset", lambda: abs(self.pose()[5]) >= 0.12,
                             2.0, "GROUND", goal, hold=0.20):
                return False

        if not self.wait("brake_release_vertical_takeoff",
                         lambda: self.handoff in ("TAKEOFF_VERTICAL",
                                                  "TAKEOFF_CLEARANCE_CONFIRM",
                                                  "AIR_HORIZONTAL_TRACK"),
                         8.0, "TAKEOFF", goal):
            return False
        if not self.wait("clearance_then_hover",
                         lambda: self.handoff == "AIR_HORIZONTAL_TRACK"
                         and abs(self.pose()[2] - 1.2) < 0.10 and self.hspeed() < 0.12,
                         15.0, "TAKEOFF", goal, hold=0.50):
            return False

        if self.scenario == "moving":
            move_goal = self.make_goal(x0 + 0.8, y0)
            if not self.wait("horizontal_move", lambda: abs(self.pose()[0] - (x0 + 0.8)) < 0.12
                             and abs(self.pose()[1] - y0) < 0.12 and self.hspeed() < 0.20,
                             12.0, "AIR", move_goal, hold=0.3):
                return False
            if not self.wait("horizontal_return", lambda: abs(self.pose()[0] - x0) < 0.12
                             and abs(self.pose()[1] - y0) < 0.12 and self.hspeed() < 0.20,
                             12.0, "AIR", goal, hold=0.3):
                return False
        else:
            if not self.wait("stable_hover", lambda: self.handoff == "AIR_HORIZONTAL_TRACK"
                             and self.hspeed() < 0.08, 4.0, "AIR", goal, hold=0.8):
                return False

        if not self.wait("land_disarm_reacquire", lambda: self.handoff == "GROUND_DRIVE"
                         and self.disarmed and mean_abs(self.actual_motor) < 5.0
                         and self.contact, 15.0, "LAND", goal, hold=0.20):
            return False

        if self.scenario == "moving":
            post_x = self.pose()[0]
            drive = Twist(); drive.linear.x = 0.45
            if not self.wait("ground_resume", lambda: self.pose()[0] >= post_x + 0.35,
                             8.0, "GROUND", goal, drive):
                return False
        self.publish("GROUND", goal, Twist())
        return True

    def result(self, sequence_passed):
        final_motor = mean_abs(self.actual_motor)
        result = {
            "scenario": self.scenario, "sequence_passed": sequence_passed,
            "stages": self.stages, "state_edges": self.state_edges,
            "finite": not self.nonfinite, "nonfinite": self.nonfinite,
            "dual_ownership_samples": self.dual_ownership_samples,
            "max_takeoff_abs_roll_rad": self.max_takeoff_roll,
            "max_takeoff_abs_pitch_rad": self.max_takeoff_pitch,
            "max_vertical_xy_drift_m": self.max_vertical_xy_drift,
            "first_horizontal_target_height_m": self.first_horizontal_height,
            "final_disarmed": self.disarmed,
            "final_actual_motor_mean_abs_rad_s": final_motor,
            "ground_reacquired": self.handoff == "GROUND_DRIVE" and self.ground_active,
            "acceptance": {
                "sequence": sequence_passed, "finite": not self.nonfinite,
                "no_dual_ownership": self.dual_ownership_samples == 0,
                "takeoff_attitude_bounded": self.max_takeoff_roll < 0.20
                                            and self.max_takeoff_pitch < 0.20,
                "vertical_xy_latched": self.max_vertical_xy_drift < 0.08,
                "clearance_before_horizontal": self.first_horizontal_height is not None
                                               and self.first_horizontal_height >= 0.366,
                "final_disarmed": self.disarmed, "final_motor_zero": final_motor < 5.0,
                "ground_reacquired": self.handoff == "GROUND_DRIVE" and self.ground_active,
            },
        }
        if self.special_acceptance is not None:
            result["acceptance"] = dict(self.special_acceptance)
            result["acceptance"]["finite"] = not self.nonfinite
            result["acceptance"]["no_dual_ownership"] = self.dual_ownership_samples == 0
        result["takeoff_site"] = {
            "measurement_valid": self.site_measurement_valid,
            "valid": self.site_valid,
            "available_m": self.site_available,
            "required_m": self.site_required,
            "nearest_m": self.site_nearest,
            "reason": self.site_reason,
            "recheck_distance_m": self.site_recheck_distance,
            "decision_reason": self.takeoff_decision_reason,
            "blocked_duration_s": self.blocked_duration,
            "blocked_distance_m": self.blocked_distance,
            "obstacle_contact_ever": self.obstacle_contact_ever,
            "max_obstacle_contact_force_n": self.max_obstacle_contact_force,
        }
        result["reposition"] = {
            "triggered": self.reposition_ever,
            "active_at_end": self.reposition_active,
            "goal_x_m": (self.reposition_goal.pose.position.x
                         if self.reposition_goal is not None else None),
            "goal_y_m": (self.reposition_goal.pose.position.y
                         if self.reposition_goal is not None else None),
            "duration_s": self.reposition_duration,
            "distance_m": self.reposition_distance,
            "initial_distance_m": self.reposition_initial_distance,
            "target_history_age_s": self.reposition_history_age,
            "result": self.reposition_result,
            "persistent_site_count": self.persistent_site_count,
            "max_persistent_site_count": self.max_persistent_site_count,
            "target_site_id": self.reposition_target_site_id,
            "target_source": self.reposition_target_source,
            "target_reachability": self.reposition_target_reachability,
            "target_revalidated": self.reposition_target_revalidated,
            "cross_sortie_candidate_seen": (
                self.persistent_cross_sortie_candidate_seen),
            "max_motor_mean_abs_rad_s": self.max_reposition_motor_mean_abs,
            "invalid_takeoff_motor_samples": self.invalid_takeoff_motor_samples,
        }
        return result

    def close(self):
        self.stream.flush(); self.stream.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=(
        "static", "moving", "yaw", "near_barrier",
        "clearance_open", "clearance_far_low", "clearance_adjacent_invalid",
        "clearance_invalid_then_move", "clearance_low_above_minimum",
        "clearance_above_cruise", "clearance_contact_stall_abort",
        "clearance_landing_handshake",
        "reposition_current_valid", "reposition_just_invalid",
        "reposition_invalid_after_move", "reposition_no_history",
        "reposition_unreachable", "reposition_persistent_cross_sortie",
    ), required=True)
    parser.add_argument("--timeline", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("uav_v4_rotors_handoff_test", anonymous=True)
    test = HandoffTest(args.scenario, args.timeline)
    passed = test.run()
    result = test.result(passed)
    test.close()
    with open(args.output, "w") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True))
    if not all(result["acceptance"].values()):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
