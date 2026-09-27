#!/usr/bin/env python3
"""Command and measure the standalone Phase-B RotorS Firefly smoke test."""

import argparse
import json
import math
import os
import statistics
import time

import rospy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


class SmokeTest:
    def __init__(self, namespace, model_name):
        self.ns = "/" + namespace.strip("/")
        self.model_name = model_name
        self.samples = []
        self.model_samples = []
        self.counts = {
            "command_pose": 0,
            "odometry": 0,
            "imu": 0,
            "controller_actuator_command": 0,
            "aggregate_motor_measurement": 0,
            "motor_0_measurement": 0,
            "gazebo_model_states": 0,
        }
        self.frames = {}
        self.last_odom = None
        self.last_model = None
        self.last_actuator_len = 0
        self.last_motor_measurement_len = 0
        self.motor0_values = []

        self.command_topic = self.ns + "/command/pose"
        self.pub = rospy.Publisher(self.command_topic, PoseStamped, queue_size=10)
        rospy.Subscriber(self.command_topic, PoseStamped, self.command_cb, queue_size=20)
        rospy.Subscriber(self.ns + "/odometry_sensor1/odometry", Odometry,
                         self.odom_cb, queue_size=100)
        rospy.Subscriber(self.ns + "/imu", Imu, self.imu_cb, queue_size=100)
        rospy.Subscriber(self.ns + "/command/motor_speed", Actuators,
                         self.actuator_cb, queue_size=100)
        rospy.Subscriber(self.ns + "/motor_speed", Actuators,
                         self.motor_measurement_cb, queue_size=100)
        rospy.Subscriber(self.ns + "/motor_speed/0", Float32,
                         self.motor0_cb, queue_size=100)
        rospy.Subscriber("/gazebo/model_states", ModelStates,
                         self.model_states_cb, queue_size=100)

    def command_cb(self, msg):
        self.counts["command_pose"] += 1
        self.frames["command_pose"] = msg.header.frame_id

    def odom_cb(self, msg):
        now = time.monotonic()
        p = msg.pose.pose.position
        v = msg.twist.twist.linear
        self.last_odom = msg
        self.samples.append((now, p.x, p.y, p.z, v.x, v.y, v.z))
        self.counts["odometry"] += 1
        self.frames["odometry_header"] = msg.header.frame_id
        self.frames["odometry_child"] = msg.child_frame_id

    def imu_cb(self, msg):
        self.counts["imu"] += 1
        self.frames["imu"] = msg.header.frame_id

    def actuator_cb(self, msg):
        self.counts["controller_actuator_command"] += 1
        self.last_actuator_len = len(msg.angular_velocities)

    def motor_measurement_cb(self, msg):
        self.counts["aggregate_motor_measurement"] += 1
        self.last_motor_measurement_len = len(msg.angular_velocities)

    def motor0_cb(self, msg):
        self.counts["motor_0_measurement"] += 1
        self.motor0_values.append(float(msg.data))

    def model_states_cb(self, msg):
        try:
            i = msg.name.index(self.model_name)
        except ValueError:
            return
        p = msg.pose[i].position
        self.last_model = (p.x, p.y, p.z)
        self.model_samples.append((time.monotonic(), p.x, p.y, p.z))
        self.counts["gazebo_model_states"] += 1

    def wait_ready(self, timeout=30.0):
        end = time.monotonic() + timeout
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < end:
            if self.last_odom is not None and self.counts["imu"] > 0 and self.last_model:
                return True
            rate.sleep()
        return False

    def make_pose(self, x, y, z):
        msg = PoseStamped()
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = 1.0
        return msg

    def run_stage(self, name, target, duration, settle_window=2.0):
        start = time.monotonic()
        end = start + duration
        msg = self.make_pose(*target)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and time.monotonic() < end:
            msg.header.stamp = rospy.Time.now()
            self.pub.publish(msg)
            rate.sleep()
        cutoff = end - settle_window
        points = [s for s in self.samples if s[0] >= cutoff]
        if not points:
            return {"name": name, "target": target, "error": "no odometry samples"}
        xs, ys, zs = ([p[i] for p in points] for i in (1, 2, 3))
        speeds = [math.sqrt(p[4] ** 2 + p[5] ** 2 + p[6] ** 2) for p in points]
        final = points[-1]
        position_error = math.sqrt(
            (final[1] - target[0]) ** 2 +
            (final[2] - target[1]) ** 2 +
            (final[3] - target[2]) ** 2)
        return {
            "name": name,
            "target": list(target),
            "duration_wall_s": duration,
            "settle_samples": len(points),
            "final_position": [final[1], final[2], final[3]],
            "final_position_error_m": position_error,
            "mean_position": [statistics.mean(xs), statistics.mean(ys), statistics.mean(zs)],
            "z_std_m": statistics.pstdev(zs) if len(zs) > 1 else 0.0,
            "mean_speed_mps": statistics.mean(speeds),
            "max_speed_mps": max(speeds),
        }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--namespace", default="firefly")
    ap.add_argument("--model-name", default="firefly")
    ap.add_argument("--output", required=True)
    args = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node("rotors_firefly_smoke_test", anonymous=True)
    test = SmokeTest(args.namespace, args.model_name)
    result = {
        "backend": "rotors_firefly_standalone",
        "world": "disaster_scene_a_rotors_smoke",
        "namespace": "/" + args.namespace.strip("/"),
        "model_name": args.model_name,
        "command_contract": {
            "topic": "/" + args.namespace.strip("/") + "/command/pose",
            "type": "geometry_msgs/PoseStamped",
            "frame_id": "world",
        },
        "stages": [],
    }

    if not test.wait_ready():
        result["ready"] = False
        result["counts"] = test.counts
    else:
        result["ready"] = True
        sequence = [
            ("takeoff", (0.0, 0.0, 1.5), 10.0),
            ("hover", (0.0, 0.0, 1.5), 5.0),
            ("horizontal_move", (2.0, 0.0, 1.5), 12.0),
            ("return", (0.0, 0.0, 1.5), 12.0),
            ("landing", (0.0, 0.0, 0.15), 10.0),
        ]
        for name, target, duration in sequence:
            rospy.loginfo("Smoke stage %s -> %s", name, target)
            result["stages"].append(test.run_stage(name, target, duration))
        result["counts"] = test.counts
        result["frames"] = test.frames
        result["controller_actuator_vector_length"] = test.last_actuator_len
        result["motor_measurement_vector_length"] = test.last_motor_measurement_len
        if test.motor0_values:
            result["motor_0_measurement"] = {
                "min": min(test.motor0_values),
                "max": max(test.motor0_values),
                "last": test.motor0_values[-1],
            }
        result["final_gazebo_model_position"] = test.last_model

        by_name = {s["name"]: s for s in result["stages"]}
        result["acceptance"] = {
            "takeoff_height": by_name["takeoff"].get("final_position_error_m", 99) < 0.25,
            "stable_hover": (
                by_name["hover"].get("final_position_error_m", 99) < 0.20 and
                by_name["hover"].get("z_std_m", 99) < 0.08 and
                by_name["hover"].get("mean_speed_mps", 99) < 0.20
            ),
            "horizontal_convergence": by_name["horizontal_move"].get("final_position_error_m", 99) < 0.25,
            "return_convergence": by_name["return"].get("final_position_error_m", 99) < 0.25,
            "controlled_landing": (
                by_name["landing"].get("final_position", [0, 0, 99])[2] < 0.30 and
                by_name["landing"].get("mean_speed_mps", 99) < 0.25
            ),
            "odometry_present": test.counts["odometry"] > 0,
            "imu_present": test.counts["imu"] > 0,
            "controller_output_present": (
                test.counts["controller_actuator_command"] > 0 and
                test.last_actuator_len == 6
            ),
            "motor_measurement_present": (
                (test.counts["aggregate_motor_measurement"] > 0 and
                 test.last_motor_measurement_len == 6) or
                test.counts["motor_0_measurement"] > 0
            ),
            "gazebo_model_state_present": test.counts["gazebo_model_states"] > 0,
        }

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(result, f, indent=2, sort_keys=True)
        f.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
