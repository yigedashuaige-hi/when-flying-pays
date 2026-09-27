#!/usr/bin/env python3
"""Phase C2 gated dynamics checks for the RotorS-native uav_v4."""

import argparse
import json
import math
import os
import time

import rospy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetJointProperties
from geometry_msgs.msg import PoseStamped
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String


def finite(values):
    return all(math.isfinite(value) for value in values)


class NativeFlightTest:
    def __init__(self):
        self.odom = None
        self.imu = None
        self.models = None
        self.aggregate_motor = None
        self.individual_motors = {}
        self.raw_motor = None
        self.bad_samples = []
        self.max_abs = {"position": 0.0, "linear_speed": 0.0,
                        "angular_speed": 0.0, "motor_speed": 0.0}
        self.frames = {}
        self.counts = {"odometry": 0, "imu": 0, "model_states": 0,
                       "aggregate_motor": 0, "raw_motor": 0}
        self.command_pub = rospy.Publisher(
            "/uav_v4/command/motor_speed", Actuators, queue_size=1)
        self.pose_pub = rospy.Publisher(
            "/uav_v4/command/pose", PoseStamped, queue_size=1)
        self.mode_pub = rospy.Publisher(
            "/rescue/mode", String, queue_size=1, latch=True)
        rospy.Subscriber("/uav_v4/odometry", Odometry, self.odom_cb)
        rospy.Subscriber("/uav_v4/imu", Imu, self.imu_cb)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.models_cb)
        rospy.Subscriber("/uav_v4/motor_speed", Actuators, self.motor_cb)
        rospy.Subscriber("/uav_v4/command/motor_speed_raw", Actuators,
                         self.raw_cb)
        for index in range(4):
            rospy.Subscriber("/uav_v4/motor_speed/%d" % index, Float32,
                             self.individual_cb, callback_args=index)

    def record_bad(self, source, values):
        if not finite(values):
            self.bad_samples.append({"source": source,
                                     "values": [None if not math.isfinite(v)
                                                else v for v in values]})

    def odom_cb(self, msg):
        self.odom = msg
        self.counts["odometry"] += 1
        self.frames["odometry_header"] = msg.header.frame_id
        self.frames["odometry_child"] = msg.child_frame_id
        p = msg.pose.pose.position; q = msg.pose.pose.orientation
        v = msg.twist.twist.linear; w = msg.twist.twist.angular
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                  v.x, v.y, v.z, w.x, w.y, w.z]
        self.record_bad("odometry", values)
        self.max_abs["position"] = max(self.max_abs["position"],
                                       abs(p.x), abs(p.y), abs(p.z))
        self.max_abs["linear_speed"] = max(self.max_abs["linear_speed"],
                                           abs(v.x), abs(v.y), abs(v.z))
        self.max_abs["angular_speed"] = max(self.max_abs["angular_speed"],
                                            abs(w.x), abs(w.y), abs(w.z))

    def imu_cb(self, msg):
        self.imu = msg
        self.counts["imu"] += 1
        self.frames["imu"] = msg.header.frame_id
        q = msg.orientation; w = msg.angular_velocity
        a = msg.linear_acceleration
        self.record_bad("imu", [q.x, q.y, q.z, q.w, w.x, w.y, w.z,
                                a.x, a.y, a.z])

    def models_cb(self, msg):
        self.models = msg
        self.counts["model_states"] += 1
        if "uav_v4" in msg.name:
            index = msg.name.index("uav_v4")
            p = msg.pose[index].position; q = msg.pose[index].orientation
            v = msg.twist[index].linear; w = msg.twist[index].angular
            self.record_bad("gazebo_model_state",
                            [p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                             v.x, v.y, v.z, w.x, w.y, w.z])

    def motor_cb(self, msg):
        self.aggregate_motor = list(msg.angular_velocities)
        self.counts["aggregate_motor"] += 1
        self.record_bad("aggregate_motor", self.aggregate_motor)

    def raw_cb(self, msg):
        self.raw_motor = list(msg.angular_velocities)
        self.counts["raw_motor"] += 1
        self.record_bad("lee_raw_motor", self.raw_motor)

    def individual_cb(self, msg, index):
        self.individual_motors[index] = msg.data
        self.record_bad("motor_%d" % index, [msg.data])
        self.max_abs["motor_speed"] = max(self.max_abs["motor_speed"],
                                          abs(msg.data))

    def ready(self, timeout=30.0):
        end = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < end:
            names = self.models.name if self.models is not None else []
            if self.odom is not None and self.imu is not None and \
                    names.count("uav_v4") == 1:
                return True
            rospy.sleep(0.05)
        return False

    def position_velocity(self):
        p = self.odom.pose.pose.position; v = self.odom.twist.twist.linear
        return [p.x, p.y, p.z], [v.x, v.y, v.z]

    @staticmethod
    def pose(x, y, z):
        msg = PoseStamped(); msg.header.frame_id = "world"
        msg.pose.position.x = x; msg.pose.position.y = y
        msg.pose.position.z = z; msg.pose.orientation.w = 1.0
        return msg

    def publish_motor_for(self, speed, duration):
        msg = Actuators(); msg.angular_velocities = [speed] * 4
        end = time.monotonic() + duration; rate = rospy.Rate(100)
        while not rospy.is_shutdown() and time.monotonic() < end:
            msg.header.stamp = rospy.Time.now(); self.command_pub.publish(msg)
            rate.sleep()

    def joint_snapshot(self):
        rospy.wait_for_service("/gazebo/get_joint_properties", timeout=5.0)
        service = rospy.ServiceProxy("/gazebo/get_joint_properties",
                                     GetJointProperties)
        output = {}
        for index in range(4):
            name = "uav_v4::uav_v4/rotor_%d_joint" % index
            response = service(name)
            values = list(response.position) + list(response.rate)
            self.record_bad("joint_%d" % index, values)
            output[str(index)] = {"success": response.success,
                                  "position": list(response.position),
                                  "rate": list(response.rate)}
        return output

    def common_result(self):
        names = self.models.name if self.models is not None else []
        position, velocity = self.position_velocity() if self.odom else ([], [])
        return {"finite": not self.bad_samples,
                "non_finite_samples": self.bad_samples[:20],
                "counts": self.counts, "frames": self.frames,
                "max_abs": self.max_abs, "final_position": position,
                "final_velocity": velocity,
                "gazebo_uav_v4_count": names.count("uav_v4"),
                "firefly_present": "firefly" in names,
                "individual_motor_last": self.individual_motors,
                "aggregate_motor_last": self.aggregate_motor}

    def run_scan(self):
        result = {"mode": "fixed_speed_scan", "steps": []}
        if not self.ready():
            result["ready"] = False; result.update(self.common_result())
            return result
        result["ready"] = True
        for speed in [0.0, 300.0, 600.0, 750.0, 900.0, 1000.0]:
            before = len(self.bad_samples)
            self.publish_motor_for(speed, 1.25)
            position, velocity = self.position_velocity()
            result["steps"].append({"command_rad_s": speed,
                                    "position": position,
                                    "velocity": velocity,
                                    "finite": len(self.bad_samples) == before,
                                    "joint_state": self.joint_snapshot()})
            if self.bad_samples:
                break
        self.publish_motor_for(0.0, 0.3)
        result.update(self.common_result())
        result["passed"] = (result["ready"] and result["finite"] and
                            result["gazebo_uav_v4_count"] == 1 and
                            not result["firefly_present"] and
                            all(step["finite"] for step in result["steps"]))
        return result

    def hold_pose(self, mode, target, predicate, timeout, hold=0.0):
        end = time.monotonic() + timeout; stable = None; rate = rospy.Rate(50)
        while not rospy.is_shutdown() and time.monotonic() < end:
            target.header.stamp = rospy.Time.now()
            self.mode_pub.publish(String(data=mode)); self.pose_pub.publish(target)
            if predicate():
                stable = stable or time.monotonic()
                if time.monotonic() - stable >= hold:
                    return True
            else:
                stable = None
            if self.bad_samples:
                return False
            rate.sleep()
        return False

    def run_lee(self):
        result = {"mode": "lee_takeoff_hover", "stages": []}
        if not self.ready():
            result["ready"] = False; result.update(self.common_result())
            return result
        result["ready"] = True; p0, _ = self.position_velocity()
        target = self.pose(p0[0], p0[1], 1.0)
        takeoff = self.hold_pose("TAKEOFF", target,
            lambda: abs(self.position_velocity()[0][2] - 1.0) < 0.12 and
                    max(abs(v) for v in self.position_velocity()[1]) < 0.25,
            20.0, 0.8)
        result["stages"].append({"name": "takeoff", "passed": takeoff,
                                 "position": self.position_velocity()[0]})
        hover = False
        if takeoff:
            hover = self.hold_pose("AIR", target,
                lambda: abs(self.position_velocity()[0][2] - 1.0) < 0.10 and
                        max(abs(v) for v in self.position_velocity()[1]) < 0.12,
                10.0, 1.5)
        result["stages"].append({"name": "hover", "passed": hover,
                                 "position": self.position_velocity()[0]})
        result.update(self.common_result())
        result["passed"] = (takeoff and hover and result["finite"] and
                            result["gazebo_uav_v4_count"] == 1 and
                            not result["firefly_present"] and
                            result["counts"]["raw_motor"] > 0 and
                            result["counts"]["imu"] > 0)
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("scan", "lee"), required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("uav_v4_rotors_native_flight_test", anonymous=True)
    test = NativeFlightTest()
    result = test.run_scan() if args.mode == "scan" else test.run_lee()
    result["backend"] = "rotors"
    result["set_model_state_used"] = False
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    if not result.get("passed", False):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
