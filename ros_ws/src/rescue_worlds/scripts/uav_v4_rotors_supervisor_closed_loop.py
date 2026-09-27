#!/usr/bin/env python3
"""Real RotorS/Gazebo closed-loop tests for the frozen D2 supervisor."""

import argparse
import json
import math
import os
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32, String


MISSION_SCRIPTS = os.path.abspath(os.path.join(
    os.path.dirname(__file__), "..", "..", "rescue_mission", "scripts"
))
if MISSION_SCRIPTS not in sys.path:
    sys.path.insert(0, MISSION_SCRIPTS)

from safety_supervisor import SafetySupervisor  # noqa: E402


class ClosedLoopSupervisorTest:
    def __init__(self, case, guarded):
        self.case = case
        self.guarded = guarded
        params = rospy.get_param("/mode_switcher/rotors_safety_supervisor")
        self.supervisor = SafetySupervisor(
            params, enforce=guarded, active=False, energy_backend="rotors"
        )
        self.odom = None
        self.disarmed = True
        self.remaining_j = None
        self.consumed_j = None
        self.motor_actual = None
        self.max_motor_omega_rad_s = 0.0
        self.non_finite = []
        self.decisions = []
        self.reserve_violations = []
        self.minimum_remaining_j = float("inf")
        self.landing_start_consumed_j = None

        self.pose_pub = rospy.Publisher(
            "/uav_v4/command/pose", PoseStamped, queue_size=10
        )
        self.mode_pub = rospy.Publisher(
            "/rescue/mode", String, queue_size=10, latch=True
        )
        self.phase_pub = rospy.Publisher(
            "/rescue/energy_phase", String, queue_size=10, latch=True
        )
        self.backend_pub = rospy.Publisher(
            "/rescue/supervisor_energy_backend", String, queue_size=10,
            latch=True
        )
        self.model_pub = rospy.Publisher(
            "/rescue/supervisor_energy_model", String, queue_size=10,
            latch=True
        )
        self.predicted_pub = rospy.Publisher(
            "/rescue/supervisor_predicted_consumption_j", Float32,
            queue_size=10
        )
        self.reserve_pub = rospy.Publisher(
            "/rescue/supervisor_reserve_j", Float32, queue_size=10
        )
        self.block_pub = rospy.Publisher(
            "/rescue/supervisor_block_reason", String, queue_size=10,
            latch=True
        )
        self.override_pub = rospy.Publisher(
            "/rescue/override_reason", String, queue_size=10, latch=True
        )
        self.safe_margin_pub = rospy.Publisher(
            "/rescue/safe_landing_margin_j", Float32, queue_size=10
        )
        rospy.Subscriber("/uav_v4/odometry", Odometry, self.odom_cb)
        rospy.Subscriber("/rescue/rotors/disarmed", Bool, self.disarmed_cb)
        rospy.Subscriber("/uav_v4/motor_speed", Actuators, self.motor_cb)
        rospy.Subscriber(
            "/rescue/rotors_mechanical_remaining_j", Float32,
            self.remaining_cb
        )
        rospy.Subscriber(
            "/rescue/rotors_mechanical_consumed_j", Float32,
            self.consumed_cb
        )

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

    def disarmed_cb(self, msg):
        self.disarmed = bool(msg.data)

    def motor_cb(self, msg):
        values = list(msg.angular_velocities)
        self.motor_actual = values
        if len(values) != 4 or not self.finite(values):
            if not self.non_finite:
                self.non_finite.append({"source": "motor_speed", "values": values})
            return
        self.max_motor_omega_rad_s = max(
            self.max_motor_omega_rad_s, max(abs(value) for value in values)
        )

    def remaining_cb(self, msg):
        self.remaining_j = float(msg.data)
        self.minimum_remaining_j = min(self.minimum_remaining_j, self.remaining_j)

    def consumed_cb(self, msg):
        self.consumed_j = float(msg.data)

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

    def wait_ready(self, timeout=35.0):
        end = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < end:
            if (self.odom is not None and self.motor_actual is not None
                    and self.remaining_j is not None
                    and self.consumed_j is not None and self.disarmed
                    and self.speed() < 0.15):
                return True
            rospy.sleep(0.05)
        return False

    def publish_accounting(self):
        self.backend_pub.publish(String(data="rotors"))
        self.model_pub.publish(String(data=self.supervisor.energy_model))
        self.predicted_pub.publish(Float32(
            data=float(self.supervisor.predicted_consumption_j)
        ))
        self.reserve_pub.publish(Float32(
            data=float(self.supervisor.energy_reserve_j)
        ))
        self.block_pub.publish(String(data=self.supervisor.block_reason))
        self.override_pub.publish(String(data=self.supervisor.override_reason))
        self.safe_margin_pub.publish(Float32(
            data=float(self.supervisor.safe_landing_margin_j)
        ))

    def approve(self, proposed, current):
        approved = self.supervisor.filter(proposed, {
            "current_mode": current,
            "energy_remaining_j": self.remaining_j,
            "landing_area_safety": 1.0,
            "stuck_risk_prior": 0.0,
        })
        self.publish_accounting()
        record = {
            "current_mode": current,
            "proposed_mode": proposed,
            "approved_mode": approved,
            "remaining_j": self.remaining_j,
            "predicted_consumption_j": self.supervisor.predicted_consumption_j,
            "reserve_j": self.supervisor.energy_reserve_j,
            "block_reason": self.supervisor.block_reason,
        }
        if (not self.decisions or
                any(record[key] != self.decisions[-1][key]
                    for key in ("current_mode", "proposed_mode",
                                "approved_mode", "block_reason"))):
            self.decisions.append(record)
        if approved == "TAKEOFF":
            required = self.supervisor.takeoff_energy_required()
        elif approved == "AIR":
            required = self.supervisor.e_land_j + self.supervisor.e_margin_j
        elif approved == "LAND":
            required = self.supervisor.e_land_j
        else:
            required = 0.0
        if self.remaining_j + 1e-6 < required:
            violation = dict(record)
            violation["approved_requirement_j"] = required
            self.reserve_violations.append(violation)
        return approved

    def publish_control(self, mode, phase, target):
        self.mode_pub.publish(String(data=mode))
        self.phase_pub.publish(String(data=phase))
        target.header.stamp = rospy.Time.now()
        self.pose_pub.publish(target)

    def wait_condition(self, mode, phase, target, condition, timeout, hold=0.0):
        end = time.monotonic() + timeout
        stable_since = None
        rate = rospy.Rate(40)
        while not rospy.is_shutdown() and time.monotonic() < end:
            self.publish_control(mode, phase, target)
            if self.non_finite:
                return False
            if condition():
                stable_since = stable_since or time.monotonic()
                if time.monotonic() - stable_since >= hold:
                    return True
            else:
                stable_since = None
            rate.sleep()
        return False

    def takeoff(self, x0, y0, height=1.0):
        approved = self.approve("TAKEOFF", "GROUND")
        if approved != "TAKEOFF":
            return False, approved
        target = self.pose(x0, y0, height)
        passed = self.wait_condition(
            "TAKEOFF", "TAKEOFF", target,
            lambda: abs(self.position()[2] - height) < 0.10 and self.speed() < 0.22,
            20.0, hold=0.6,
        )
        return passed, approved

    def air_loop(self, target, duration=None, stop_on_low_margin=False):
        begin = rospy.Time.now()
        rate = rospy.Rate(40)
        observed_unguarded_violation = False
        while not rospy.is_shutdown():
            approved = self.approve("AIR", "AIR")
            if approved == "LAND":
                return "forced_land", observed_unguarded_violation
            if (self.supervisor.block_reason == "air_unsafe_force_land"
                    and approved == "AIR"):
                observed_unguarded_violation = True
                if stop_on_low_margin:
                    # Record that unguarded continued, then terminate the smoke
                    # safely by issuing an explicit test-harness landing.
                    self.publish_control("AIR", "HOVER", target)
                    rospy.sleep(0.25)
                    return "unguarded_continued", True
            self.publish_control("AIR", "HOVER", target)
            if self.non_finite:
                return "non_finite", observed_unguarded_violation
            if duration is not None and (rospy.Time.now() - begin).to_sec() >= duration:
                return "complete", observed_unguarded_violation
            rate.sleep()

    def trajectory(self, start, finish, speed):
        distance = math.hypot(finish[0] - start[0], finish[1] - start[1])
        duration = distance / speed
        begin = rospy.Time.now()
        rate = rospy.Rate(40)
        while not rospy.is_shutdown():
            approved = self.approve("AIR", "AIR")
            if approved == "LAND":
                return False, "forced_land"
            elapsed = (rospy.Time.now() - begin).to_sec()
            fraction = min(1.0, elapsed / duration)
            target = self.pose(
                start[0] + fraction * (finish[0] - start[0]),
                start[1] + fraction * (finish[1] - start[1]), finish[2]
            )
            self.publish_control("AIR", "TRANSLATION", target)
            if self.non_finite:
                return False, "non_finite"
            if fraction >= 1.0:
                break
            rate.sleep()
        target = self.pose(*finish)
        passed = self.wait_condition(
            "AIR", "TRANSLATION", target,
            lambda: (math.hypot(self.position()[0] - finish[0],
                                self.position()[1] - finish[1]) < 0.10
                     and abs(self.position()[2] - finish[2]) < 0.10
                     and self.speed() < 0.20),
            8.0, hold=0.4,
        )
        return passed, "complete" if passed else "tracking_timeout"

    def land(self, x0, y0):
        self.approve("LAND", "AIR")
        if self.consumed_j is not None:
            self.landing_start_consumed_j = self.consumed_j
        target = self.pose(x0, y0, 0.05)
        passed = self.wait_condition(
            "LAND", "LANDING", target,
            lambda: (self.disarmed and self.position()[2] <= 0.09
                     and self.speed() < 0.16),
            22.0, hold=0.3,
        )
        if passed:
            self.publish_control("GROUND", "GROUND", target)
            rospy.sleep(0.5)
        return passed

    def probe_preflight(self):
        approved = self.approve("TAKEOFF", "GROUND")
        x0, y0, _ = self.position()
        target = self.pose(x0, y0, 0.05)
        end = time.monotonic() + 2.0
        while not rospy.is_shutdown() and time.monotonic() < end:
            self.publish_control("GROUND", "GROUND", target)
            rospy.sleep(0.05)
        return approved

    def run(self):
        if not self.wait_ready():
            return {"passed": False, "failure": "topics_not_ready"}
        initial_remaining = self.remaining_j
        initial_consumed = self.consumed_j
        x0, y0, _ = self.position()
        details = {}

        if self.case in ("preflight_insufficient", "boundary_exact",
                         "boundary_below"):
            approved = self.probe_preflight()
            expected = "TAKEOFF" if self.case == "boundary_exact" else "GROUND"
            details["preflight_approved_mode"] = approved
            passed = (approved == expected and self.disarmed
                      and self.max_motor_omega_rad_s < 1.0
                      and self.position()[2] < 0.10)
        else:
            takeoff_passed, approved = self.takeoff(x0, y0)
            details["takeoff_approved_mode"] = approved
            details["takeoff_passed"] = takeoff_passed
            passed = takeoff_passed
            air_outcome = "not_run"
            unguarded_continued = False
            if passed and self.case == "sufficient":
                target = self.pose(x0, y0, 1.0)
                air_outcome, _ = self.air_loop(target, duration=2.0)
                if air_outcome == "complete":
                    finish = (x0 + 0.6, y0, 1.0)
                    outbound, outbound_reason = self.trajectory(
                        (x0, y0, 1.0), finish, 0.30
                    )
                    returned, return_reason = (False, "not_run")
                    if outbound:
                        returned, return_reason = self.trajectory(
                            finish, (x0, y0, 1.0), 0.30
                        )
                    details.update(outbound_passed=outbound,
                                   outbound_reason=outbound_reason,
                                   return_passed=returned,
                                   return_reason=return_reason)
                    passed = outbound and returned
                else:
                    passed = False
            elif passed and self.case in ("airborne_low_guarded",
                                           "airborne_low_unguarded"):
                target = self.pose(x0, y0, 1.0)
                air_outcome, unguarded_continued = self.air_loop(
                    target, duration=None, stop_on_low_margin=True
                )
                expected = ("forced_land" if self.guarded
                            else "unguarded_continued")
                passed = air_outcome == expected
            details["air_outcome"] = air_outcome
            details["unguarded_continued_after_low_margin"] = unguarded_continued
            prelanding_remaining = self.remaining_j
            landing_passed = self.land(x0, y0) if takeoff_passed else False
            details["prelanding_remaining_j"] = prelanding_remaining
            details["landing_passed"] = landing_passed
            passed = passed and landing_passed

        final_consumed = self.consumed_j
        landing_energy = None
        if self.landing_start_consumed_j is not None and final_consumed is not None:
            landing_energy = final_consumed - self.landing_start_consumed_j
        result = {
            "date": "2026-07-20",
            "case": self.case,
            "guarded": self.guarded,
            "backend": "rotors",
            "energy_model": self.supervisor.energy_model,
            "passed": bool(passed and not self.non_finite),
            "initial_remaining_j": initial_remaining,
            "final_remaining_j": self.remaining_j,
            "minimum_remaining_j": self.minimum_remaining_j,
            "initial_consumed_j": initial_consumed,
            "final_consumed_j": final_consumed,
            "landing_actual_energy_j": landing_energy,
            "final_position": list(self.position()),
            "final_speed_m_s": self.speed(),
            "final_disarmed": self.disarmed,
            "max_motor_omega_rad_s": self.max_motor_omega_rad_s,
            "reserve_violation_count": len(self.reserve_violations),
            "reserve_violations": self.reserve_violations,
            "decisions": self.decisions,
            "non_finite_samples": self.non_finite,
            "details": details,
            "tolerance_or_hysteresis_added": False,
        }
        return result


def parse_bool(value):
    value = value.strip().lower()
    if value in ("true", "1", "yes"):
        return True
    if value in ("false", "0", "no"):
        return False
    raise argparse.ArgumentTypeError("expected true/false")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", required=True, choices=(
        "sufficient", "preflight_insufficient", "airborne_low_guarded",
        "airborne_low_unguarded", "boundary_exact", "boundary_below",
    ))
    parser.add_argument("--guarded", type=parse_bool, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(rospy.myargv()[1:])
    rospy.init_node("uav_v4_rotors_supervisor_closed_loop")
    runner = ClosedLoopSupervisorTest(args.case, args.guarded)
    result = runner.run()
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w") as stream:
        json.dump(result, stream, indent=2, sort_keys=True, allow_nan=False)
        stream.write("\n")
    print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False))
    if not result["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
