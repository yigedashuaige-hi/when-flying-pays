#!/usr/bin/env python3
"""Run the small, predeclared Phase-D1 calibration/held-out maneuver set."""

import argparse
import json
import math
import os
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


TRIALS = [
    {"id": "cal_01", "split": "calibration", "hover_s": 2.0,
     "distance_m": 0.0, "target_speed_m_s": 0.0},
    {"id": "cal_02", "split": "calibration", "hover_s": 4.0,
     "distance_m": 0.0, "target_speed_m_s": 0.0},
    {"id": "cal_03", "split": "calibration", "hover_s": 6.0,
     "distance_m": 0.0, "target_speed_m_s": 0.0},
    {"id": "cal_04", "split": "calibration", "hover_s": 2.0,
     "distance_m": 0.6, "target_speed_m_s": 0.25},
    {"id": "cal_05", "split": "calibration", "hover_s": 4.0,
     "distance_m": 1.0, "target_speed_m_s": 0.35},
    {"id": "cal_06", "split": "calibration", "hover_s": 3.0,
     "distance_m": 1.4, "target_speed_m_s": 0.50},
    {"id": "heldout_01", "split": "heldout", "hover_s": 3.0,
     "distance_m": 0.0, "target_speed_m_s": 0.0},
    {"id": "heldout_02", "split": "heldout", "hover_s": 3.0,
     "distance_m": 0.8, "target_speed_m_s": 0.30},
    {"id": "heldout_03", "split": "heldout", "hover_s": 5.0,
     "distance_m": 1.2, "target_speed_m_s": 0.45},
]


class CalibrationRunner:
    def __init__(self):
        self.odom = None
        self.disarmed = True
        self.motor_actual = None
        self.non_finite = []
        self.pose_pub = rospy.Publisher(
            "/uav_v4/command/pose", PoseStamped, queue_size=10
        )
        self.mode_pub = rospy.Publisher(
            "/rescue/mode", String, queue_size=10, latch=True
        )
        self.phase_pub = rospy.Publisher(
            "/rescue/energy_phase", String, queue_size=10, latch=True
        )
        self.trial_pub = rospy.Publisher(
            "/rescue/calibration_trial_id", String, queue_size=10, latch=True
        )
        self.split_pub = rospy.Publisher(
            "/rescue/calibration_split", String, queue_size=10, latch=True
        )
        rospy.Subscriber("/uav_v4/odometry", Odometry, self.odom_cb)
        rospy.Subscriber("/rescue/rotors/disarmed", Bool, self.disarmed_cb)
        rospy.Subscriber("/uav_v4/motor_speed", Actuators, self.motor_cb)

    @staticmethod
    def finite(values):
        return all(math.isfinite(value) for value in values)

    def odom_cb(self, msg):
        self.odom = msg
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                  v.x, v.y, v.z, w.x, w.y, w.z]
        if not self.finite(values) and not self.non_finite:
            self.non_finite.append({"source": "odometry", "values": values})

    def motor_cb(self, msg):
        self.motor_actual = list(msg.angular_velocities)
        if (len(self.motor_actual) != 4 or
                not self.finite(self.motor_actual)) and not self.non_finite:
            self.non_finite.append(
                {"source": "motor_speed", "values": self.motor_actual})

    def disarmed_cb(self, msg):
        self.disarmed = msg.data

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

    def publish(self, mode, phase, target):
        self.mode_pub.publish(String(data=mode))
        self.phase_pub.publish(String(data=phase))
        target.header.stamp = rospy.Time.now()
        self.pose_pub.publish(target)

    def wait_ready(self, timeout=30.0):
        end = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < end:
            if (self.odom is not None and self.motor_actual is not None and
                    self.disarmed and self.speed() < 0.12):
                return True
            rospy.sleep(0.05)
        return False

    def wait_condition(self, mode, phase, target, condition, timeout, hold=0.0):
        end = time.monotonic() + timeout
        stable = None
        rate = rospy.Rate(40)
        while not rospy.is_shutdown() and time.monotonic() < end:
            self.publish(mode, phase, target)
            if self.non_finite:
                return False
            if condition():
                stable = stable or time.monotonic()
                if time.monotonic() - stable >= hold:
                    return True
            else:
                stable = None
            rate.sleep()
        return False

    def timed_hold(self, phase, target, duration):
        start = rospy.Time.now()
        rate = rospy.Rate(40)
        while not rospy.is_shutdown() and (rospy.Time.now() - start).to_sec() < duration:
            self.publish("AIR", phase, target)
            if self.non_finite:
                return False
            rate.sleep()
        return True

    def trajectory(self, phase, start, finish, speed):
        distance = math.hypot(finish[0] - start[0], finish[1] - start[1])
        if distance <= 1e-9:
            return True
        duration = distance / speed
        begin = rospy.Time.now()
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - begin).to_sec()
            fraction = min(1.0, elapsed / duration)
            target = self.pose(
                start[0] + fraction * (finish[0] - start[0]),
                start[1] + fraction * (finish[1] - start[1]),
                finish[2],
            )
            self.publish("AIR", phase, target)
            if self.non_finite:
                return False
            if fraction >= 1.0:
                break
            rate.sleep()
        target = self.pose(*finish)
        return self.wait_condition(
            "AIR", phase, target,
            lambda: (math.hypot(self.position()[0] - finish[0],
                                self.position()[1] - finish[1]) < 0.10 and
                     abs(self.position()[2] - finish[2]) < 0.10 and
                     self.speed() < 0.18),
            8.0, hold=0.4,
        )

    def run_trial(self, spec, early_stop_after=""):
        self.trial_pub.publish(String(data=spec["id"]))
        self.split_pub.publish(String(data=spec["split"]))
        self.phase_pub.publish(String(data="GROUND"))
        self.mode_pub.publish(String(data="GROUND"))
        rospy.sleep(0.5)
        x0, y0, _ = self.position()
        target_height_m = float(spec.get("height_m", 1.0))
        target = self.pose(x0, y0, target_height_m)
        stages = []

        passed = self.wait_condition(
            "TAKEOFF", "TAKEOFF", target,
            lambda: (abs(self.position()[2] - target_height_m) < 0.10 and
                     self.speed() < 0.22),
            18.0, hold=0.6,
        )
        stages.append({"name": "takeoff", "passed": passed,
                       "position": list(self.position())})
        if not passed or early_stop_after == "takeoff":
            return passed, stages

        passed = self.timed_hold("HOVER", target, spec["hover_s"])
        stages.append({"name": "hover", "passed": passed,
                       "duration_s": spec["hover_s"]})
        if not passed or early_stop_after == "hover":
            return passed, stages

        if spec["distance_m"] > 0.0:
            finish = (x0 + spec["distance_m"], y0, target_height_m)
            passed = self.trajectory(
                "TRANSLATION", (x0, y0, target_height_m), finish,
                spec["target_speed_m_s"])
            stages.append({"name": "translation", "passed": passed,
                           "target": list(finish)})
            if not passed:
                return False, stages
            passed = self.trajectory(
                "RETURN", finish, (x0, y0, target_height_m),
                spec["target_speed_m_s"])
            stages.append({"name": "return", "passed": passed,
                           "target": [x0, y0, target_height_m]})
            if not passed:
                return False, stages

        land_target = self.pose(x0, y0, 0.05)
        passed = self.wait_condition(
            "LAND", "LANDING", land_target,
            lambda: (self.disarmed and self.position()[2] <= 0.09 and
                     self.speed() < 0.16),
            20.0, hold=0.3,
        )
        stages.append({"name": "landing_disarm", "passed": passed,
                       "position": list(self.position()),
                       "disarmed": self.disarmed})
        if passed:
            self.publish("GROUND", "GROUND", self.pose(x0, y0, 0.05))
            rospy.sleep(0.5)
        return passed, stages


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--max-trials", type=int, default=len(TRIALS))
    parser.add_argument("--early-stop-after", choices=("", "takeoff", "hover"),
                        default="")
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("uav_v4_rotors_energy_calibration", anonymous=True)
    runner = CalibrationRunner()
    result = {
        "date": "2026-07-18",
        "backend": "rotors",
        "energy_model": "rotors_aero_shaft_mechanical_proxy_v1",
        "plan": TRIALS[:args.max_trials],
        "trials": [],
        "early_stop_after": args.early_stop_after,
    }
    if not runner.wait_ready():
        result["passed"] = False
        result["failure"] = "topics_not_ready"
    else:
        result["passed"] = True
        for spec in TRIALS[:args.max_trials]:
            passed, stages = runner.run_trial(spec, args.early_stop_after)
            result["trials"].append(
                {"id": spec["id"], "split": spec["split"],
                 "passed": passed, "stages": stages}
            )
            if not passed or args.early_stop_after:
                result["passed"] = False if not passed else True
                break
    result["non_finite_samples"] = runner.non_finite
    result["all_values_finite"] = not runner.non_finite
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    if not result.get("passed"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
