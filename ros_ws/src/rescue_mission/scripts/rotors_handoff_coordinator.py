#!/usr/bin/env python3
"""Measured, single-owner ground/flight handoff for the RotorS backend."""

import math
import os
import sys
from collections import deque

import rospy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, WrenchStamped
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, Int32, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
from safe_site_memory import SafeSiteMemory


class RotorsHandoffCoordinator:
    GROUND_DRIVE = "GROUND_DRIVE"
    TAKEOFF_REPOSITION = "TAKEOFF_REPOSITION"
    TAKEOFF_REPOSITION_FAILED = "TAKEOFF_REPOSITION_FAILED"
    PRE_TAKEOFF_BRAKE = "PRE_TAKEOFF_BRAKE"
    RELEASE_AND_ARM = "RELEASE_AND_ARM"
    TAKEOFF_VERTICAL = "TAKEOFF_VERTICAL"
    TAKEOFF_CLEARANCE_CONFIRM = "TAKEOFF_CLEARANCE_CONFIRM"
    AIR_HORIZONTAL_TRACK = "AIR_HORIZONTAL_TRACK"
    LAND_VERTICAL = "LAND_VERTICAL"
    TOUCHDOWN_SETTLE = "TOUCHDOWN_SETTLE"
    DISARM_CONFIRM = "DISARM_CONFIRM"
    GROUND_REACQUIRE = "GROUND_REACQUIRE"

    def __init__(self):
        gp = rospy.get_param
        self.rate_hz = float(gp("~update_rate", 50.0))
        self.cruise_altitude = float(gp("~cruise_altitude", 1.2))
        self.ground_reference = float(gp("~ground_reference_height", 0.066))
        self.ground_command_altitude = float(gp("~ground_command_altitude", 0.05))
        self.brake_speed = float(gp("~brake_horizontal_speed", 0.05))
        self.brake_attitude = float(gp("~brake_attitude_limit", 0.10))
        self.brake_yaw_rate = float(gp("~brake_yaw_rate", 0.10))
        self.brake_motor_speed = float(gp("~brake_motor_speed", 5.0))
        self.brake_hold = float(gp("~brake_hold_sec", 0.30))
        self.release_zero_updates = int(gp("~release_zero_updates", 3))
        self.release_min = float(gp("~release_min_sec", 0.03))
        self.zero_force = float(gp("~ground_wrench_zero_force", 0.001))
        self.zero_torque = float(gp("~ground_wrench_zero_torque", 0.0001))
        self.clearance_height = float(gp("~clearance_height", 0.30))
        self.target_altitude_tolerance = float(gp("~target_altitude_tolerance", 0.10))
        self.clearance_vz = float(gp("~clearance_vertical_speed", 0.20))
        self.clearance_attitude = float(gp("~clearance_attitude_limit", 0.15))
        self.clearance_hold = float(gp("~clearance_hold_sec", 0.20))
        self.touchdown_height_tolerance = float(gp("~touchdown_height_tolerance", 0.025))
        self.touchdown_vz = float(gp("~touchdown_vertical_speed", 0.10))
        self.touchdown_hspeed = float(gp("~touchdown_horizontal_speed", 0.08))
        self.touchdown_attitude = float(gp("~touchdown_attitude_limit", 0.15))
        self.touchdown_hold = float(gp("~touchdown_hold_sec", 0.40))
        self.disarm_motor_speed = float(gp("~disarm_motor_speed", 5.0))
        self.disarm_hold = float(gp("~disarm_hold_sec", 0.20))
        self.reacquire_hold = float(gp("~reacquire_hold_sec", 0.10))
        self.site_measurement_timeout = float(gp("~site_measurement_timeout_sec", 0.15))
        self.site_preflight_hold = float(gp("~site_preflight_hold_sec", 0.20))
        self.contact_abort_hold = float(gp("~contact_abort_hold_sec", 0.10))
        self.takeoff_progress_epsilon = float(gp("~takeoff_progress_epsilon_m", 0.025))
        self.takeoff_stall_timeout = float(gp("~takeoff_stall_timeout_sec", 1.0))
        self.takeoff_stall_grace = float(gp("~takeoff_stall_grace_sec", 0.75))
        self.history_window = float(gp("~valid_site_history_window_sec", 12.0))
        self.history_sample_period = float(gp("~valid_site_history_sample_period_sec", 0.10))
        self.reposition_min_separation = float(gp("~reposition_min_separation_m", 0.30))
        self.reposition_goal_tolerance = float(gp("~reposition_goal_tolerance_m", 0.03))
        self.reposition_speed_tolerance = float(gp("~reposition_speed_tolerance_m_s", 0.05))
        self.reposition_timeout = float(gp("~reposition_timeout_sec", 8.0))
        self.reposition_no_progress_timeout = float(gp("~reposition_no_progress_timeout_sec", 2.0))
        self.reposition_progress_epsilon = float(gp("~reposition_progress_epsilon_m", 0.005))
        self.persistent_site_capacity = int(gp("~persistent_safe_site_capacity", 32))
        self.persistent_site_dedup = float(
            gp("~persistent_safe_site_dedup_distance_m", 0.15))

        self.requested_mode = "GROUND"
        self.state = self.GROUND_DRIVE
        self.state_since = rospy.Time.now()
        self.stable_since = None
        self.odom = None
        self.goal = None
        self.actual_motor_mean_abs = 0.0
        self.disarmed = True
        self.ground_active = False
        self.ground_wrench_force = float("inf")
        self.ground_wrench_torque = float("inf")
        self.release_zero_count = 0
        self.latched_x = 0.0
        self.latched_y = 0.0
        self.latched_yaw = 0.0
        self.site_measurement_valid = False
        self.site_valid = False
        self.site_stamp = rospy.Time(0)
        self.site_available = float("nan")
        self.site_required = float("nan")
        self.site_nearest = float("nan")
        self.site_reason = "measurement_missing"
        self.site_valid_since = None
        self.site_recheck_distance = float("inf")
        self.obstacle_contact = False
        self.obstacle_contact_force = 0.0
        self.contact_since = None
        self.takeoff_max_z = float("-inf")
        self.takeoff_last_progress = rospy.Time(0)
        self.abort_active = False
        self.takeoff_blocked = False
        self.blocked_since = None
        self.blocked_origin = None
        self.blocked_last_xy = None
        self.blocked_path_distance = 0.0
        self.blocked_site_state = None
        self.decision_reason = "init"
        self.site_history = deque()
        self.site_memory = SafeSiteMemory(
            capacity=self.persistent_site_capacity,
            dedup_distance_m=self.persistent_site_dedup,
        )
        self.last_history_sample = rospy.Time(0)
        self.ground_epoch = 0
        self.recovery_episode_id = 0
        self.reposition_target = None
        self.reposition_since = None
        self.reposition_origin = None
        self.reposition_last_xy = None
        self.reposition_distance = 0.0
        self.reposition_final_duration = 0.0
        self.reposition_best_goal_distance = float("inf")
        self.reposition_initial_goal_distance = 0.0
        self.reposition_target_history_age = 0.0
        self.reposition_last_progress = rospy.Time(0)
        self.reposition_result = "none"
        self.reposition_restored_pose = None
        self.reposition_target_site_id = 0
        self.reposition_target_source = "none"
        self.reposition_target_reachability = "none"
        self.reposition_target_observation_age = 0.0
        self.reposition_target_clearance_available = float("nan")
        self.reposition_target_clearance_required = float("nan")
        self.reposition_target_nearest = float("nan")
        self.reposition_target_revalidated = False

        self.mode_pub = rospy.Publisher("/rescue/mode", String, queue_size=10, latch=True)
        self.state_pub = rospy.Publisher(
            "/rescue/rotors/handoff_state", String, queue_size=10, latch=True
        )
        self.owner_pub = rospy.Publisher(
            "/rescue/rotors/ground_ownership", Bool, queue_size=10, latch=True
        )
        self.motor_enable_pub = rospy.Publisher(
            "/rescue/rotors/motor_enable", Bool, queue_size=10, latch=True
        )
        self.contact_pub = rospy.Publisher(
            "/rescue/rotors/contact_inferred", Bool, queue_size=10, latch=True
        )
        self.decision_reason_pub = rospy.Publisher(
            "/rescue/rotors/takeoff_decision_reason", String, queue_size=10, latch=True
        )
        self.blocked_duration_pub = rospy.Publisher(
            "/rescue/rotors/takeoff_blocked_duration_s", Float32, queue_size=10
        )
        self.blocked_distance_pub = rospy.Publisher(
            "/rescue/rotors/takeoff_blocked_distance_m", Float32, queue_size=10
        )
        self.approved_takeoff_pose_pub = rospy.Publisher(
            "/rescue/rotors/approved_takeoff_pose", PoseStamped,
            queue_size=1, latch=True,
        )
        self.reposition_goal_pub = rospy.Publisher(
            "/rescue/rotors/reposition_goal", PoseStamped, queue_size=1, latch=True
        )
        self.reposition_active_pub = rospy.Publisher(
            "/rescue/rotors/reposition_active", Bool, queue_size=10, latch=True
        )
        self.reposition_duration_pub = rospy.Publisher(
            "/rescue/rotors/reposition_duration_s", Float32, queue_size=10
        )
        self.reposition_distance_pub = rospy.Publisher(
            "/rescue/rotors/reposition_distance_m", Float32, queue_size=10
        )
        self.reposition_result_pub = rospy.Publisher(
            "/rescue/rotors/reposition_result", String, queue_size=10, latch=True
        )
        self.reposition_initial_distance_pub = rospy.Publisher(
            "/rescue/rotors/reposition_initial_distance_m", Float32, queue_size=10
        )
        self.reposition_history_age_pub = rospy.Publisher(
            "/rescue/rotors/reposition_target_history_age_s", Float32, queue_size=10
        )
        self.reposition_restored_pose_pub = rospy.Publisher(
            "/rescue/rotors/reposition_site_restored_pose", PoseStamped,
            queue_size=1, latch=True,
        )
        self.persistent_site_count_pub = rospy.Publisher(
            "/rescue/rotors/persistent_safe_site_count", Int32,
            queue_size=10, latch=True,
        )
        self.reposition_target_site_id_pub = rospy.Publisher(
            "/rescue/rotors/reposition_target_site_id", Int32,
            queue_size=10, latch=True,
        )
        self.reposition_target_source_pub = rospy.Publisher(
            "/rescue/rotors/reposition_target_source", String,
            queue_size=10, latch=True,
        )
        self.reposition_target_reachability_pub = rospy.Publisher(
            "/rescue/rotors/reposition_target_reachability", String,
            queue_size=10, latch=True,
        )
        self.reposition_target_observation_age_pub = rospy.Publisher(
            "/rescue/rotors/reposition_target_observation_age_s", Float32,
            queue_size=10,
        )
        self.reposition_target_clearance_available_pub = rospy.Publisher(
            "/rescue/rotors/reposition_target_clearance_available_m", Float32,
            queue_size=10,
        )
        self.reposition_target_clearance_required_pub = rospy.Publisher(
            "/rescue/rotors/reposition_target_clearance_required_m", Float32,
            queue_size=10,
        )
        self.reposition_target_nearest_pub = rospy.Publisher(
            "/rescue/rotors/reposition_target_nearest_obstacle_m", Float32,
            queue_size=10,
        )
        self.reposition_target_revalidated_pub = rospy.Publisher(
            "/rescue/rotors/reposition_target_revalidated", Bool,
            queue_size=10, latch=True,
        )
        self.air_goal_pub = rospy.Publisher("/rescue/air_goal", PoseStamped, queue_size=10)
        self.command_pub = rospy.Publisher("/uav_v4/command/pose", PoseStamped, queue_size=10)

        rospy.Subscriber("/rescue/requested_mode", String, self.request_cb, queue_size=1)
        rospy.Subscriber("/rescue/goal", PoseStamped, self.goal_cb, queue_size=1)
        rospy.Subscriber("/uav_v4/odometry", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/uav_v4/motor_speed", Actuators, self.motor_cb, queue_size=1)
        rospy.Subscriber("/rescue/rotors/disarmed", Bool, self.disarmed_cb, queue_size=1)
        rospy.Subscriber(
            "/rescue/rotors/ground_control_active", Bool, self.ground_active_cb, queue_size=1
        )
        rospy.Subscriber(
            "/rescue/rotors/ground_control_wrench", WrenchStamped,
            self.ground_wrench_cb, queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/takeoff_site_status", DiagnosticArray,
            self.site_status_cb, queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/obstacle_contact", Bool,
            self.obstacle_contact_cb, queue_size=1,
        )
        rospy.Subscriber(
            "/rescue/rotors/obstacle_contact_force_n", Float32,
            self.obstacle_contact_force_cb, queue_size=1,
        )

    def request_cb(self, msg):
        if msg.data in ("GROUND", "TAKEOFF", "AIR", "LAND"):
            self.requested_mode = msg.data

    def goal_cb(self, msg):
        self.goal = msg

    def odom_cb(self, msg):
        self.odom = msg
        if self.takeoff_blocked:
            p = msg.pose.pose.position
            current = (p.x, p.y)
            if self.blocked_last_xy is not None:
                self.blocked_path_distance += math.hypot(
                    current[0] - self.blocked_last_xy[0],
                    current[1] - self.blocked_last_xy[1],
                )
            self.blocked_last_xy = current

    def motor_cb(self, msg):
        values = list(msg.angular_velocities)
        if values and all(math.isfinite(value) for value in values):
            self.actual_motor_mean_abs = sum(abs(value) for value in values) / len(values)
        else:
            self.actual_motor_mean_abs = float("inf")

    def disarmed_cb(self, msg):
        self.disarmed = bool(msg.data)

    def ground_active_cb(self, msg):
        self.ground_active = bool(msg.data)

    def ground_wrench_cb(self, msg):
        f = msg.wrench.force
        t = msg.wrench.torque
        self.ground_wrench_force = math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
        self.ground_wrench_torque = math.sqrt(t.x * t.x + t.y * t.y + t.z * t.z)
        if self.state == self.RELEASE_AND_ARM and self.ground_zero():
            self.release_zero_count += 1
        elif self.state == self.RELEASE_AND_ARM:
            self.release_zero_count = 0

    def site_status_cb(self, msg):
        if not msg.status:
            return
        values = {item.key: item.value for item in msg.status[0].values}
        try:
            measurement_valid = values["measurement_valid"] == "true"
            site_valid = values["takeoff_site_valid"] == "true"
            available = float(values["vertical_clearance_available_m"])
            required = float(values["vertical_clearance_required_m"])
            nearest = float(values["nearest_obstacle_distance_m"])
            recheck = max(0.0, float(values["recheck_distance_m"]))
        except (KeyError, ValueError):
            self.site_measurement_valid = False
            self.site_valid = False
            self.site_reason = "measurement_invalid:malformed_atomic_status"
            self.site_valid_since = None
            return
        now = rospy.Time.now()
        if measurement_valid and site_valid:
            if not (self.site_measurement_valid and self.site_valid):
                self.site_valid_since = now
        else:
            self.site_valid_since = None
        self.site_stamp = msg.header.stamp
        self.site_measurement_valid = measurement_valid
        self.site_valid = site_valid
        self.site_available = available
        self.site_required = required
        self.site_nearest = nearest
        self.site_recheck_distance = recheck
        self.site_reason = msg.status[0].message

    def obstacle_contact_cb(self, msg):
        self.obstacle_contact = bool(msg.data)
        now = rospy.Time.now()
        if self.obstacle_contact:
            self.contact_since = self.contact_since or now
        else:
            self.contact_since = None

    def obstacle_contact_force_cb(self, msg):
        self.obstacle_contact_force = max(0.0, float(msg.data))

    def attitude(self):
        q = self.odom.pose.pose.orientation
        return euler_from_quaternion([q.x, q.y, q.z, q.w])

    def hspeed(self):
        v = self.odom.twist.twist.linear
        return math.hypot(v.x, v.y)

    def motor_zero(self, limit):
        return math.isfinite(self.actual_motor_mean_abs) and self.actual_motor_mean_abs <= limit

    def stable_for(self, condition, duration):
        now = rospy.Time.now()
        if not condition:
            self.stable_since = None
            return False
        if self.stable_since is None:
            self.stable_since = now
            return False
        return (now - self.stable_since).to_sec() >= duration

    def enter(self, state):
        if state == self.state:
            return
        previous = self.state
        rospy.loginfo("RotorS handoff: %s -> %s", previous, state)
        self.state = state
        self.state_since = rospy.Time.now()
        self.stable_since = None
        self.release_zero_count = 0
        if previous == self.GROUND_REACQUIRE and state == self.GROUND_DRIVE:
            self.ground_epoch += 1
        if state == self.TAKEOFF_VERTICAL and self.odom is not None:
            self.takeoff_max_z = self.odom.pose.pose.position.z
            self.takeoff_last_progress = rospy.Time.now()

    def latch_current_pose(self):
        p = self.odom.pose.pose.position
        self.latched_x = p.x
        self.latched_y = p.y
        self.latched_yaw = self.attitude()[2]

    def brake_ok(self):
        if self.odom is None:
            return False
        roll, pitch, _ = self.attitude()
        yaw_rate = self.odom.twist.twist.angular.z
        return (self.hspeed() < self.brake_speed
                and abs(roll) < self.brake_attitude
                and abs(pitch) < self.brake_attitude
                and abs(yaw_rate) < self.brake_yaw_rate
                and self.motor_zero(self.brake_motor_speed))

    def ground_zero(self):
        return (not self.ground_active
                and self.ground_wrench_force <= self.zero_force
                and self.ground_wrench_torque <= self.zero_torque)

    def clearance_ok(self):
        if self.odom is None or self.ground_active or not self.site_ready():
            return False
        p = self.odom.pose.pose.position
        v = self.odom.twist.twist.linear
        roll, pitch, _ = self.attitude()
        return (p.z - self.ground_reference >= self.clearance_height
                and p.z >= self.cruise_altitude - self.target_altitude_tolerance
                and abs(v.z) <= self.clearance_vz
                and abs(roll) < self.clearance_attitude
                and abs(pitch) < self.clearance_attitude
                and not self.obstacle_contact)

    def site_measurement_fresh(self):
        return (not self.site_stamp.is_zero()
                and (rospy.Time.now() - self.site_stamp).to_sec()
                <= self.site_measurement_timeout)

    def site_ready(self):
        return (self.site_measurement_fresh()
                and self.site_measurement_valid and self.site_valid)

    def site_preflight_ready(self):
        return (self.site_ready() and self.site_valid_since is not None
                and (rospy.Time.now() - self.site_valid_since).to_sec()
                >= self.site_preflight_hold)

    def site_state(self):
        return (self.site_measurement_valid, self.site_valid, self.site_reason)

    def history_eligible(self):
        public_mode, owner, motor_enable = self.outputs()
        return (public_mode == "GROUND" and owner and not motor_enable
                and self.site_measurement_fresh())

    def remember_safe_site(self, source="stable_ground"):
        """Persist only fresh, currently stable-valid grounded evidence."""
        eligible = self.history_eligible()
        if source == "approved_takeoff":
            _, owner, motor_enable = self.outputs()
            eligible = (owner and not motor_enable
                        and self.state == self.PRE_TAKEOFF_BRAKE
                        and self.site_measurement_fresh())
        if (self.odom is None or not eligible or not self.site_preflight_ready()):
            return None
        now = rospy.Time.now()
        p = self.odom.pose.pose.position
        return self.site_memory.upsert({
            "x": p.x,
            "y": p.y,
            "yaw": self.attitude()[2],
            "observation_stamp": self.site_stamp.to_sec(),
            "last_validated_stamp": now.to_sec(),
            "measurement_valid": self.site_measurement_valid,
            "site_valid": self.site_valid,
            "stable_valid": True,
            "available_m": self.site_available,
            "required_m": self.site_required,
            "nearest_m": self.site_nearest,
            "recheck_distance_m": self.site_recheck_distance,
            "source": source,
            "ground_epoch": self.ground_epoch,
        })

    def record_site_history(self):
        if self.odom is None or not self.history_eligible():
            return
        now = rospy.Time.now()
        if (not self.last_history_sample.is_zero()
                and (now - self.last_history_sample).to_sec()
                < self.history_sample_period):
            return
        p = self.odom.pose.pose.position
        memory_site_id = self.remember_safe_site()
        self.site_history.append({
            "stamp": now,
            "x": p.x,
            "y": p.y,
            "yaw": self.attitude()[2],
            "measurement_valid": self.site_measurement_valid,
            "site_valid": self.site_valid,
            "stable_valid": self.site_preflight_ready(),
            "site_age": max(0.0, (now - self.site_stamp).to_sec()),
            "nearest": self.site_nearest,
            "available": self.site_available,
            "required": self.site_required,
            "recheck": self.site_recheck_distance,
            "memory_site_id": memory_site_id or 0,
        })
        self.last_history_sample = now
        cutoff = (now - rospy.Duration(self.history_window)
                  if now.to_sec() >= self.history_window else rospy.Time(0))
        while self.site_history and self.site_history[0]["stamp"] < cutoff:
            self.site_history.popleft()

    def select_reposition_target(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        for sample in reversed(self.site_history):
            separation = math.hypot(p.x - sample["x"], p.y - sample["y"])
            if (sample["measurement_valid"] and sample["stable_valid"]
                    and sample["site_age"] <= self.site_measurement_timeout
                    and separation >= self.reposition_min_separation):
                target = dict(sample)
                target.update({
                    "source": "recent_reverse_path",
                    "site_id": sample.get("memory_site_id", 0),
                    "reachability": "observed_ground_path",
                    "observation_stamp": sample["stamp"].to_sec(),
                    "available_m": sample.get("available", float("nan")),
                    "required_m": sample.get("required", float("nan")),
                    "nearest_m": sample["nearest"],
                })
                return target
        target = self.site_memory.select(
            current_xy=(p.x, p.y),
            episode_id=self.recovery_episode_id,
            ground_epoch=self.ground_epoch,
            min_separation_m=self.reposition_min_separation,
        )
        if target is None:
            return None
        target["stamp"] = rospy.Time.from_sec(target["observation_stamp"])
        target["nearest"] = target["nearest_m"]
        return target

    def reposition_goal_message(self):
        if self.reposition_target is None:
            return None
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.position.x = self.reposition_target["x"]
        msg.pose.position.y = self.reposition_target["y"]
        msg.pose.position.z = self.ground_command_altitude
        q = quaternion_from_euler(0.0, 0.0, self.reposition_target["yaw"])
        msg.pose.orientation.x, msg.pose.orientation.y = q[0], q[1]
        msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
        return msg

    def start_reposition(self):
        self.begin_blocked_episode()
        target = self.select_reposition_target()
        if target is None:
            self.reposition_target = None
            self.reposition_target_site_id = 0
            self.reposition_target_source = "none"
            self.reposition_target_reachability = "none"
            self.reposition_target_revalidated = False
            self.reposition_result = "takeoff_reposition_failed:no_stable_valid_history"
            self.decision_reason = self.reposition_result
            self.enter(self.TAKEOFF_REPOSITION_FAILED)
            rospy.logerr("RotorS %s", self.reposition_result)
            return
        now = rospy.Time.now()
        p = self.odom.pose.pose.position
        self.reposition_target = target
        self.reposition_target_site_id = int(target.get("site_id", 0))
        self.reposition_target_source = target.get("source", "recent_reverse_path")
        self.reposition_target_reachability = target.get(
            "reachability", "observed_ground_path")
        self.reposition_target_observation_age = max(
            0.0, now.to_sec() - float(target.get(
                "observation_stamp", target["stamp"].to_sec())))
        self.reposition_target_clearance_available = float(
            target.get("available_m", float("nan")))
        self.reposition_target_clearance_required = float(
            target.get("required_m", float("nan")))
        self.reposition_target_nearest = float(
            target.get("nearest_m", target["nearest"]))
        self.reposition_target_revalidated = False
        self.reposition_since = now
        self.reposition_origin = (p.x, p.y)
        self.reposition_last_xy = (p.x, p.y)
        self.reposition_distance = 0.0
        self.reposition_final_duration = 0.0
        self.reposition_best_goal_distance = math.hypot(
            p.x - target["x"], p.y - target["y"])
        self.reposition_initial_goal_distance = self.reposition_best_goal_distance
        self.reposition_target_history_age = max(0.0, (now - target["stamp"]).to_sec())
        self.reposition_last_progress = now
        self.reposition_result = "takeoff_reposition_active"
        self.decision_reason = self.reposition_result
        rospy.logwarn(
            "RotorS deferred takeoff reposition: request=(%.3f,%.3f) "
            "target=(%.3f,%.3f) source=%s site_id=%d reachability=%s "
            "history_age=%.3f s separation=%.3f m nearest=%.3f m",
            p.x, p.y, target["x"], target["y"],
            self.reposition_target_source, self.reposition_target_site_id,
            self.reposition_target_reachability,
            (now - target["stamp"]).to_sec(),
            self.reposition_best_goal_distance, target["nearest"])
        self.enter(self.TAKEOFF_REPOSITION)

    def finish_reposition_failure(self, reason):
        if self.reposition_target_site_id > 0:
            self.site_memory.mark_unreachable(
                self.reposition_target_site_id, self.recovery_episode_id)
            self.reposition_target_reachability = "unreachable_current_episode"
        self.reposition_final_duration = (
            (rospy.Time.now() - self.reposition_since).to_sec()
            if self.reposition_since else 0.0)
        self.reposition_result = "takeoff_reposition_failed:%s" % reason
        self.decision_reason = self.reposition_result
        rospy.logerr("RotorS %s after %.3f s / %.3f m",
                     self.reposition_result,
                     ((rospy.Time.now() - self.reposition_since).to_sec()
                      if self.reposition_since else 0.0),
                     self.reposition_distance)
        self.enter(self.TAKEOFF_REPOSITION_FAILED)

    def reposition_update(self):
        now = rospy.Time.now()
        p = self.odom.pose.pose.position
        current = (p.x, p.y)
        if self.reposition_last_xy is not None:
            self.reposition_distance += math.hypot(
                current[0] - self.reposition_last_xy[0],
                current[1] - self.reposition_last_xy[1])
        self.reposition_last_xy = current
        goal_distance = math.hypot(
            p.x - self.reposition_target["x"],
            p.y - self.reposition_target["y"])
        if goal_distance <= self.reposition_best_goal_distance - self.reposition_progress_epsilon:
            self.reposition_best_goal_distance = goal_distance
            self.reposition_last_progress = now
        elapsed = (now - self.reposition_since).to_sec()
        if elapsed >= self.reposition_timeout:
            self.finish_reposition_failure("timeout")
        elif (goal_distance > self.reposition_goal_tolerance
              and (now - self.reposition_last_progress).to_sec()
              >= self.reposition_no_progress_timeout):
            self.finish_reposition_failure("no_progress")
        elif (goal_distance <= self.reposition_goal_tolerance
              and self.hspeed() <= self.reposition_speed_tolerance
              and self.site_preflight_ready()):
            self.reposition_target_revalidated = True
            if self.reposition_target_site_id > 0:
                self.site_memory.mark_reached(
                    self.reposition_target_site_id, self.ground_epoch)
                self.reposition_target_reachability = "verified_current_epoch"
            self.remember_safe_site(source="stable_ground")
            self.reposition_restored_pose = (p.x, p.y, self.attitude()[2])
            restored = PoseStamped()
            restored.header.stamp = now
            restored.header.frame_id = "world"
            restored.pose.position.x = p.x
            restored.pose.position.y = p.y
            restored.pose.position.z = p.z
            q = quaternion_from_euler(0.0, 0.0, self.reposition_restored_pose[2])
            restored.pose.orientation.x, restored.pose.orientation.y = q[0], q[1]
            restored.pose.orientation.z, restored.pose.orientation.w = q[2], q[3]
            self.reposition_restored_pose_pub.publish(restored)
            self.reposition_final_duration = elapsed
            self.reposition_result = "takeoff_reposition_succeeded"
            self.end_blocked_episode(self.reposition_result)
            self.decision_reason = self.reposition_result
            self.enter(self.PRE_TAKEOFF_BRAKE)

    def blocked_reason(self):
        if not self.site_measurement_fresh():
            return "takeoff_blocked:measurement_stale"
        if not self.site_measurement_valid:
            return "takeoff_blocked:measurement_invalid"
        if not self.site_valid:
            return "takeoff_blocked:%s" % (self.site_reason or "site_invalid")
        return "takeoff_blocked:recheck_not_satisfied"

    def begin_blocked_episode(self):
        if self.takeoff_blocked or self.odom is None:
            return
        self.recovery_episode_id += 1
        p = self.odom.pose.pose.position
        self.takeoff_blocked = True
        self.blocked_since = rospy.Time.now()
        self.blocked_origin = (p.x, p.y)
        self.blocked_last_xy = (p.x, p.y)
        self.blocked_path_distance = 0.0
        self.blocked_site_state = self.site_state()
        self.decision_reason = self.blocked_reason()
        rospy.logwarn("RotorS %s at (%.3f, %.3f); required=%.3f available=%.3f nearest=%.3f",
                      self.decision_reason, p.x, p.y, self.site_required,
                      self.site_available, self.site_nearest)

    def end_blocked_episode(self, reason):
        if not self.takeoff_blocked:
            return
        p = self.odom.pose.pose.position if self.odom is not None else None
        duration = ((rospy.Time.now() - self.blocked_since).to_sec()
                    if self.blocked_since is not None else 0.0)
        rospy.loginfo("RotorS takeoff block ended: %s; duration=%.3f s distance=%.3f m final=(%s,%s)",
                      reason, duration, self.blocked_path_distance,
                      "%.3f" % p.x if p else "nan", "%.3f" % p.y if p else "nan")
        self.takeoff_blocked = False
        self.blocked_since = None
        self.blocked_origin = None
        self.blocked_last_xy = None
        self.blocked_site_state = None
        self.decision_reason = reason

    def takeoff_retry_ready(self):
        if not self.site_preflight_ready():
            return False
        if not self.takeoff_blocked:
            return True
        geometry_changed = self.site_state() != self.blocked_site_state
        moved_enough = self.blocked_path_distance >= self.site_recheck_distance
        return geometry_changed or moved_enough

    def publish_approved_takeoff_pose(self):
        self.remember_safe_site(source="approved_takeoff")
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.position.x = self.latched_x
        msg.pose.position.y = self.latched_y
        msg.pose.position.z = self.odom.pose.pose.position.z
        q = quaternion_from_euler(0.0, 0.0, self.latched_yaw)
        msg.pose.orientation.x, msg.pose.orientation.y = q[0], q[1]
        msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
        self.approved_takeoff_pose_pub.publish(msg)

    def contact_sustained(self):
        return (self.contact_since is not None
                and (rospy.Time.now() - self.contact_since).to_sec()
                >= self.contact_abort_hold)

    def takeoff_stalled(self):
        if self.odom is None:
            return False
        now = rospy.Time.now()
        z = self.odom.pose.pose.position.z
        if z >= self.takeoff_max_z + self.takeoff_progress_epsilon:
            self.takeoff_max_z = z
            self.takeoff_last_progress = now
        return ((now - self.state_since).to_sec() >= self.takeoff_stall_grace
                and (now - self.takeoff_last_progress).to_sec()
                >= self.takeoff_stall_timeout
                and z < self.cruise_altitude - self.target_altitude_tolerance)

    def begin_takeoff_abort(self, reason):
        self.abort_active = True
        self.decision_reason = "takeoff_abort:%s" % reason
        self.latch_current_pose()
        rospy.logerr("RotorS %s; commanding measured-feedback vertical landing",
                     self.decision_reason)
        self.enter(self.LAND_VERTICAL)

    def touchdown_geometry_ok(self):
        if self.odom is None:
            return False
        p = self.odom.pose.pose.position
        v = self.odom.twist.twist.linear
        roll, pitch, _ = self.attitude()
        return (p.z <= self.ground_reference + self.touchdown_height_tolerance
                and abs(v.z) <= self.touchdown_vz
                and self.hspeed() <= self.touchdown_hspeed
                and abs(roll) < self.touchdown_attitude
                and abs(pitch) < self.touchdown_attitude)

    def update(self):
        if self.odom is None:
            return
        self.record_site_history()
        wants_air = self.requested_mode in ("TAKEOFF", "AIR")
        wants_land = self.requested_mode in ("GROUND", "LAND")

        if self.state == self.GROUND_DRIVE:
            if wants_air:
                if self.takeoff_retry_ready():
                    self.end_blocked_episode("takeoff_approved_after_recheck")
                    self.decision_reason = "takeoff_approved:site_clear"
                    self.enter(self.PRE_TAKEOFF_BRAKE)
                else:
                    self.start_reposition()
            elif self.takeoff_blocked:
                self.end_blocked_episode("takeoff_request_withdrawn")

        elif self.state == self.TAKEOFF_REPOSITION:
            if wants_land:
                self.reposition_final_duration = (
                    (rospy.Time.now() - self.reposition_since).to_sec()
                    if self.reposition_since else 0.0)
                self.reposition_result = "takeoff_reposition_cancelled:request_withdrawn"
                self.decision_reason = self.reposition_result
                self.end_blocked_episode(self.reposition_result)
                self.enter(self.GROUND_DRIVE)
            else:
                self.reposition_update()

        elif self.state == self.TAKEOFF_REPOSITION_FAILED:
            if wants_land:
                self.end_blocked_episode("takeoff_reposition_failure_acknowledged")
                self.reposition_target = None
                self.enter(self.GROUND_DRIVE)
            elif self.site_preflight_ready() and self.takeoff_retry_ready():
                self.remember_safe_site(source="stable_ground")
                self.reposition_target = None
                self.reposition_target_site_id = 0
                self.reposition_target_source = "current_site"
                self.reposition_target_reachability = "verified_current_epoch"
                self.reposition_target_revalidated = True
                self.reposition_result = (
                    "takeoff_reposition_succeeded:current_site_revalidated")
                self.end_blocked_episode(self.reposition_result)
                self.decision_reason = "takeoff_approved:current_site_revalidated"
                self.enter(self.PRE_TAKEOFF_BRAKE)

        elif self.state == self.PRE_TAKEOFF_BRAKE:
            if wants_land:
                self.enter(self.GROUND_DRIVE)
            elif not self.site_ready():
                self.decision_reason = self.blocked_reason()
                self.enter(self.GROUND_DRIVE)
            elif self.stable_for(self.brake_ok(), self.brake_hold):
                self.latch_current_pose()
                self.publish_approved_takeoff_pose()
                self.enter(self.RELEASE_AND_ARM)

        elif self.state == self.RELEASE_AND_ARM:
            if wants_land:
                self.enter(self.GROUND_REACQUIRE)
            elif not self.site_ready():
                self.abort_active = True
                self.decision_reason = "takeoff_abort:site_invalid_before_arm"
                self.enter(self.GROUND_REACQUIRE)
            elif (self.ground_zero()
                  and self.release_zero_count >= self.release_zero_updates
                  and (rospy.Time.now() - self.state_since).to_sec() >= self.release_min):
                self.enter(self.TAKEOFF_VERTICAL)

        elif self.state == self.TAKEOFF_VERTICAL:
            if wants_land:
                self.latch_current_pose()
                self.enter(self.LAND_VERTICAL)
            elif not self.site_ready():
                self.begin_takeoff_abort(self.blocked_reason())
            elif self.contact_sustained():
                self.begin_takeoff_abort("sustained_obstacle_contact")
            elif self.takeoff_stalled():
                self.begin_takeoff_abort("height_progress_stall")
            elif self.clearance_ok():
                self.enter(self.TAKEOFF_CLEARANCE_CONFIRM)

        elif self.state == self.TAKEOFF_CLEARANCE_CONFIRM:
            if wants_land:
                self.latch_current_pose()
                self.enter(self.LAND_VERTICAL)
            elif not self.site_ready():
                self.begin_takeoff_abort(self.blocked_reason())
            elif self.contact_sustained():
                self.begin_takeoff_abort("sustained_obstacle_contact")
            elif not self.clearance_ok():
                self.enter(self.TAKEOFF_VERTICAL)
            elif self.stable_for(True, self.clearance_hold):
                self.enter(self.AIR_HORIZONTAL_TRACK)

        elif self.state == self.AIR_HORIZONTAL_TRACK:
            if wants_land:
                self.latch_current_pose()
                self.enter(self.LAND_VERTICAL)

        elif self.state == self.LAND_VERTICAL:
            if self.touchdown_geometry_ok():
                self.enter(self.TOUCHDOWN_SETTLE)

        elif self.state == self.TOUCHDOWN_SETTLE:
            if not self.touchdown_geometry_ok():
                self.enter(self.LAND_VERTICAL)
            elif self.stable_for(True, self.touchdown_hold):
                self.enter(self.DISARM_CONFIRM)

        elif self.state == self.DISARM_CONFIRM:
            condition = (self.disarmed and self.motor_zero(self.disarm_motor_speed)
                         and self.touchdown_geometry_ok())
            if self.stable_for(condition, self.disarm_hold):
                self.enter(self.GROUND_REACQUIRE)

        elif self.state == self.GROUND_REACQUIRE:
            condition = (self.disarmed and self.motor_zero(self.disarm_motor_speed)
                         and self.touchdown_geometry_ok())
            if not condition:
                self.enter(self.DISARM_CONFIRM)
            elif (rospy.Time.now() - self.state_since).to_sec() >= self.reacquire_hold:
                self.abort_active = False
                self.enter(self.GROUND_DRIVE)

    def outputs(self):
        ground_owner = self.state in (
            self.GROUND_DRIVE, self.TAKEOFF_REPOSITION,
            self.TAKEOFF_REPOSITION_FAILED, self.PRE_TAKEOFF_BRAKE,
            self.GROUND_REACQUIRE
        )
        motor_enable = self.state in (
            self.TAKEOFF_VERTICAL, self.TAKEOFF_CLEARANCE_CONFIRM,
            self.AIR_HORIZONTAL_TRACK, self.LAND_VERTICAL, self.TOUCHDOWN_SETTLE,
        )
        if self.state in (self.GROUND_DRIVE, self.TAKEOFF_REPOSITION,
                          self.TAKEOFF_REPOSITION_FAILED,
                          self.GROUND_REACQUIRE):
            public_mode = "GROUND"
        elif self.state in (self.PRE_TAKEOFF_BRAKE, self.RELEASE_AND_ARM,
                            self.TAKEOFF_VERTICAL, self.TAKEOFF_CLEARANCE_CONFIRM):
            public_mode = "TAKEOFF"
        elif self.state == self.AIR_HORIZONTAL_TRACK:
            public_mode = "AIR"
        else:
            public_mode = "LAND"
        return public_mode, ground_owner, motor_enable

    def command_pose(self):
        if self.odom is None:
            return None
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        if self.state == self.AIR_HORIZONTAL_TRACK and self.goal is not None:
            msg.pose = self.goal.pose
            msg.pose.position.z = self.cruise_altitude
            return msg
        if self.state in (self.LAND_VERTICAL, self.TOUCHDOWN_SETTLE,
                          self.DISARM_CONFIRM, self.GROUND_REACQUIRE):
            z = self.ground_command_altitude
        elif self.state in (self.TAKEOFF_VERTICAL, self.TAKEOFF_CLEARANCE_CONFIRM):
            z = self.cruise_altitude
        else:
            p = self.odom.pose.pose.position
            self.latched_x, self.latched_y = p.x, p.y
            self.latched_yaw = self.attitude()[2]
            z = self.ground_command_altitude
        msg.pose.position.x = self.latched_x
        msg.pose.position.y = self.latched_y
        msg.pose.position.z = z
        q = quaternion_from_euler(0.0, 0.0, self.latched_yaw)
        msg.pose.orientation.x, msg.pose.orientation.y = q[0], q[1]
        msg.pose.orientation.z, msg.pose.orientation.w = q[2], q[3]
        return msg

    def publish(self):
        public_mode, owner, motor_enable = self.outputs()
        self.mode_pub.publish(String(data=public_mode))
        self.state_pub.publish(String(data=self.state))
        self.owner_pub.publish(Bool(data=owner))
        self.motor_enable_pub.publish(Bool(data=motor_enable))
        self.contact_pub.publish(Bool(data=self.touchdown_geometry_ok()))
        self.decision_reason_pub.publish(String(data=self.decision_reason))
        blocked_duration = ((rospy.Time.now() - self.blocked_since).to_sec()
                            if self.blocked_since is not None else 0.0)
        self.blocked_duration_pub.publish(Float32(data=blocked_duration))
        self.blocked_distance_pub.publish(Float32(data=self.blocked_path_distance))
        reposition_active = self.state == self.TAKEOFF_REPOSITION
        recovery_goal = self.reposition_goal_message()
        if recovery_goal is not None:
            self.reposition_goal_pub.publish(recovery_goal)
        self.reposition_active_pub.publish(Bool(data=reposition_active))
        reposition_duration = (
            (rospy.Time.now() - self.reposition_since).to_sec()
            if reposition_active and self.reposition_since is not None
            else self.reposition_final_duration
        )
        self.reposition_duration_pub.publish(Float32(data=reposition_duration))
        self.reposition_distance_pub.publish(Float32(data=self.reposition_distance))
        self.reposition_result_pub.publish(String(data=self.reposition_result))
        self.reposition_initial_distance_pub.publish(Float32(
            data=self.reposition_initial_goal_distance))
        self.reposition_history_age_pub.publish(Float32(
            data=self.reposition_target_history_age))
        self.persistent_site_count_pub.publish(Int32(data=len(self.site_memory)))
        self.reposition_target_site_id_pub.publish(Int32(
            data=self.reposition_target_site_id))
        self.reposition_target_source_pub.publish(String(
            data=self.reposition_target_source))
        self.reposition_target_reachability_pub.publish(String(
            data=self.reposition_target_reachability))
        self.reposition_target_observation_age_pub.publish(Float32(
            data=self.reposition_target_observation_age))
        self.reposition_target_clearance_available_pub.publish(Float32(
            data=self.reposition_target_clearance_available))
        self.reposition_target_clearance_required_pub.publish(Float32(
            data=self.reposition_target_clearance_required))
        self.reposition_target_nearest_pub.publish(Float32(
            data=self.reposition_target_nearest))
        self.reposition_target_revalidated_pub.publish(Bool(
            data=self.reposition_target_revalidated))
        pose = self.command_pose()
        if pose is not None:
            self.air_goal_pub.publish(pose)
            if self.state not in (self.GROUND_DRIVE, self.TAKEOFF_REPOSITION,
                                  self.TAKEOFF_REPOSITION_FAILED,
                                  self.PRE_TAKEOFF_BRAKE):
                self.command_pub.publish(pose)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.update()
            self.publish()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rotors_handoff_coordinator")
    try:
        RotorsHandoffCoordinator().spin()
    except rospy.ROSInterruptException:
        pass
