#!/usr/bin/env python3
"""Measured-feedback Phase-C smoke for one uav_v4 model."""

import argparse
import json
import math
import os
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped, Twist
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, String


def json_safe(value):
    """Represent non-finite simulator values as JSON null, recursively."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return value


class HybridSmoke:
    def __init__(self):
        self.odom = None
        self.disarmed = True
        self.flight_state = ""
        self.model_names = []
        self.counts = {
            "command_pose": 0,
            "odometry": 0,
            "imu": 0,
            "lee_raw_output": 0,
            "gated_motor_command": 0,
            "aggregate_motor_speed": 0,
            "motor_0_speed": 0,
            "mode_state": 0,
            "gazebo_model_states": 0,
        }
        self.frames = {}
        self.vector_lengths = {}
        self.stage_results = []
        self.non_finite_samples = []

        self.pose_pub = rospy.Publisher("/rescue/goal", PoseStamped, queue_size=10)
        self.mode_pub = rospy.Publisher(
            "/rescue/requested_mode", String, queue_size=10, latch=True
        )
        self.ground_pub = rospy.Publisher(
            "/mecanum/cmd_vel", Twist, queue_size=10
        )

        rospy.Subscriber("/uav_v4/command/pose", PoseStamped, self.pose_cb)
        rospy.Subscriber("/uav_v4/odometry", Odometry, self.odom_cb)
        rospy.Subscriber("/uav_v4/imu", Imu, self.imu_cb)
        rospy.Subscriber(
            "/uav_v4/command/motor_speed_raw", Actuators, self.raw_cb
        )
        rospy.Subscriber(
            "/uav_v4/command/motor_speed", Actuators, self.gated_cb
        )
        rospy.Subscriber("/uav_v4/motor_speed", Actuators, self.motor_cb)
        rospy.Subscriber("/uav_v4/motor_speed/0", Float32, self.motor0_cb)
        rospy.Subscriber("/rescue/rotors/disarmed", Bool, self.disarmed_cb)
        rospy.Subscriber(
            "/rescue/rotors/flight_state", String, self.flight_state_cb
        )
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.models_cb)

    def pose_cb(self, msg):
        self.counts["command_pose"] += 1
        self.frames["command_pose"] = msg.header.frame_id

    def odom_cb(self, msg):
        self.odom = msg
        self.counts["odometry"] += 1
        self.frames["odometry_header"] = msg.header.frame_id
        self.frames["odometry_child"] = msg.child_frame_id
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        self.check_finite("odometry", [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                                       v.x, v.y, v.z, w.x, w.y, w.z])

    def imu_cb(self, msg):
        self.counts["imu"] += 1
        self.frames["imu"] = msg.header.frame_id
        q = msg.orientation
        w = msg.angular_velocity
        a = msg.linear_acceleration
        self.check_finite("imu", [q.x, q.y, q.z, q.w, w.x, w.y, w.z,
                                  a.x, a.y, a.z])

    def raw_cb(self, msg):
        self.counts["lee_raw_output"] += 1
        self.vector_lengths["lee_raw_output"] = len(msg.angular_velocities)
        self.check_finite("lee_raw_output", msg.angular_velocities)

    def gated_cb(self, msg):
        self.counts["gated_motor_command"] += 1
        self.vector_lengths["gated_motor_command"] = len(msg.angular_velocities)
        self.vector_lengths["gated_last"] = list(msg.angular_velocities)
        self.check_finite("gated_motor_command", msg.angular_velocities)

    def motor_cb(self, msg):
        self.counts["aggregate_motor_speed"] += 1
        self.vector_lengths["aggregate_motor_speed"] = len(msg.angular_velocities)
        self.vector_lengths["aggregate_last"] = list(msg.angular_velocities)
        self.check_finite("aggregate_motor_speed", msg.angular_velocities)

    def motor0_cb(self, _msg):
        self.counts["motor_0_speed"] += 1

    def disarmed_cb(self, msg):
        self.disarmed = msg.data

    def flight_state_cb(self, msg):
        self.flight_state = msg.data
        self.counts["mode_state"] += 1

    def models_cb(self, msg):
        self.model_names = list(msg.name)
        self.counts["gazebo_model_states"] += 1
        if "uav_v4" in msg.name:
            index = msg.name.index("uav_v4")
            p = msg.pose[index].position
            q = msg.pose[index].orientation
            v = msg.twist[index].linear
            w = msg.twist[index].angular
            self.check_finite("gazebo_model_state",
                              [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                               v.x, v.y, v.z, w.x, w.y, w.z])

    def check_finite(self, source, values):
        if self.non_finite_samples:
            return
        if not all(math.isfinite(value) for value in values):
            self.non_finite_samples.append({
                "source": source,
                "values": [value if math.isfinite(value) else None
                           for value in values],
            })

    def position(self):
        p = self.odom.pose.pose.position
        return p.x, p.y, p.z

    def speed(self):
        v = self.odom.twist.twist.linear
        return math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)

    @staticmethod
    def pose(x, y, z):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        return msg

    def publish_controls(self, mode, pose=None, twist=None):
        self.mode_pub.publish(String(data=mode))
        if pose is not None:
            pose.header.stamp = rospy.Time.now()
            self.pose_pub.publish(pose)
        self.ground_pub.publish(twist if twist is not None else Twist())

    def wait_condition(self, name, condition, timeout, mode, pose=None, twist=None,
                       hold_sec=0.0):
        start = time.monotonic()
        stable_since = None
        rate = rospy.Rate(30)
        passed = False
        while not rospy.is_shutdown() and time.monotonic() - start < timeout:
            self.publish_controls(mode, pose, twist)
            if self.non_finite_samples:
                break
            if condition():
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= hold_sec:
                    passed = True
                    break
            else:
                stable_since = None
            rate.sleep()
        snapshot = {
            "name": name,
            "passed": passed,
            "elapsed_wall_s": time.monotonic() - start,
            "mode": mode,
            "flight_state": self.flight_state,
            "disarmed": self.disarmed,
        }
        if self.odom is not None:
            snapshot["position"] = list(self.position())
            snapshot["speed_mps"] = self.speed()
        self.stage_results.append(snapshot)
        rospy.loginfo("Hybrid smoke %s: %s", name, "PASS" if passed else "FAIL")
        return passed

    def wait_ready(self):
        return self.wait_condition(
            "ready_ground_settled",
            lambda: (
                self.odom is not None
                and self.counts["imu"] > 0
                and "uav_v4" in self.model_names
                and self.disarmed
                and self.speed() < 0.10
            ),
            30.0,
            "GROUND",
            hold_sec=0.5,
        )

    def run(self):
        if not self.wait_ready():
            return False
        start_x, start_y, _ = self.position()

        ground_cmd = Twist()
        ground_cmd.linear.x = 0.18
        if not self.wait_condition(
            "ground_drive_before_flight",
            lambda: self.position()[0] >= start_x + 0.40,
            8.0,
            "GROUND",
            twist=ground_cmd,
        ):
            return False
        ground_x, ground_y, _ = self.position()

        takeoff_pose = self.pose(ground_x, ground_y, 1.2)
        if not self.wait_condition(
            "takeoff_measured",
            lambda: abs(self.position()[2] - 1.2) < 0.12 and self.speed() < 0.25,
            25.0,
            "TAKEOFF",
            takeoff_pose,
            hold_sec=0.8,
        ):
            return False
        if not self.wait_condition(
            "stable_hover",
            lambda: abs(self.position()[2] - 1.2) < 0.10 and self.speed() < 0.12,
            8.0,
            "AIR",
            takeoff_pose,
            hold_sec=1.0,
        ):
            return False

        move_pose = self.pose(ground_x + 0.80, ground_y, 1.2)
        if not self.wait_condition(
            "horizontal_flight",
            lambda: (
                abs(self.position()[0] - (ground_x + 0.80)) < 0.12
                and abs(self.position()[1] - ground_y) < 0.12
                and self.speed() < 0.20
            ),
            15.0,
            "AIR",
            move_pose,
            hold_sec=0.8,
        ):
            return False

        return_pose = self.pose(ground_x, ground_y, 1.2)
        if not self.wait_condition(
            "return_flight",
            lambda: (
                abs(self.position()[0] - ground_x) < 0.12
                and abs(self.position()[1] - ground_y) < 0.12
                and self.speed() < 0.20
            ),
            15.0,
            "AIR",
            return_pose,
            hold_sec=0.8,
        ):
            return False

        land_pose = self.pose(ground_x, ground_y, 0.05)
        if not self.wait_condition(
            "landing_touchdown_disarm",
            lambda: self.disarmed and self.position()[2] <= 0.09 and self.speed() < 0.16,
            20.0,
            "LAND",
            land_pose,
            hold_sec=0.3,
        ):
            return False

        post_x = self.position()[0]
        if not self.wait_condition(
            "ground_drive_after_flight",
            lambda: self.position()[0] >= post_x + 0.35,
            8.0,
            "GROUND",
            twist=ground_cmd,
        ):
            return False
        self.publish_controls("GROUND")
        return True


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("uav_v4_rotors_hybrid_smoke_test", anonymous=True)
    test = HybridSmoke()
    sequence_passed = test.run()
    result = {
        "backend": "rotors",
        "model": "uav_v4",
        "world": "disaster_scene_a",
        "sequence_passed": sequence_passed,
        "stages": test.stage_results,
        "counts": test.counts,
        "frames": test.frames,
        "vector_lengths_and_final_values": test.vector_lengths,
        "gazebo_uav_v4_count": test.model_names.count("uav_v4"),
        "firefly_present": "firefly" in test.model_names,
        "final_disarmed": test.disarmed,
        "non_finite_samples": test.non_finite_samples,
        "topic_contract": {
            "command_pose": "/uav_v4/command/pose geometry_msgs/PoseStamped world",
            "odometry": "/uav_v4/odometry nav_msgs/Odometry world -> uav_v4/base_link",
            "imu": "/uav_v4/imu sensor_msgs/Imu uav_v4/base_link",
            "lee_output": "/uav_v4/command/motor_speed_raw mav_msgs/Actuators",
            "gated_motor_command": "/uav_v4/command/motor_speed mav_msgs/Actuators",
            "motor_speed": "/uav_v4/motor_speed mav_msgs/Actuators",
            "mode_state": "/rescue/rotors/flight_state std_msgs/String",
        },
    }
    result["acceptance"] = {
        "all_measured_stages": sequence_passed,
        "one_uav_v4": result["gazebo_uav_v4_count"] == 1,
        "no_firefly": not result["firefly_present"],
        "odometry": test.counts["odometry"] > 0,
        "imu": test.counts["imu"] > 0,
        "lee_output": test.counts["lee_raw_output"] > 0,
        "gated_motor_command": test.counts["gated_motor_command"] > 0,
        "motor_measurement": (
            test.counts["aggregate_motor_speed"] > 0
            or test.counts["motor_0_speed"] > 0
        ),
        "final_disarmed": test.disarmed,
        "finite_throughout": not test.non_finite_samples,
    }
    result = json_safe(result)
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as stream:
        json.dump(result, stream, indent=2, sort_keys=True)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
