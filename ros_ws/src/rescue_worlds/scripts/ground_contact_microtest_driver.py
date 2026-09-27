#!/usr/bin/env python3
"""Publish deterministic GROUND/disarmed inputs for the E1a microtest."""

import argparse
import math
import time

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=4.0)
    parser.add_argument("--settle", type=float, default=0.5)
    parser.add_argument("--vx", type=float, default=0.45)
    parser.add_argument("--vy", type=float, default=0.0)
    parser.add_argument("--wz", type=float, default=0.0)
    parser.add_argument("--release-at", type=float, default=-1.0,
                        help="sim seconds; switch to TAKEOFF/armed to prove release")
    return parser.parse_args(rospy.myargv()[1:])


def main():
    rospy.init_node("ground_contact_microtest_driver")
    args = parse_args()
    mode_pub = rospy.Publisher("/rescue/mode", String, queue_size=1, latch=True)
    armed_pub = rospy.Publisher(
        "/rescue/rotors/armed", Bool, queue_size=1, latch=True
    )
    disarmed_pub = rospy.Publisher(
        "/rescue/rotors/disarmed", Bool, queue_size=1, latch=True
    )
    cmd_pub = rospy.Publisher("/mecanum/cmd_vel", Twist, queue_size=1)

    rospy.wait_for_service("/gazebo/unpause_physics", timeout=30.0)
    deadline = time.monotonic() + 1.0
    zero = Twist()
    while time.monotonic() < deadline and not rospy.is_shutdown():
        mode_pub.publish(String(data="GROUND"))
        armed_pub.publish(Bool(data=False))
        disarmed_pub.publish(Bool(data=True))
        cmd_pub.publish(zero)
        time.sleep(0.02)
    rospy.ServiceProxy("/gazebo/unpause_physics", Empty)()

    start = rospy.Time.now()
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start).to_sec()
        if elapsed >= args.duration:
            break
        command = Twist()
        if elapsed >= args.settle:
            command.linear.x = args.vx
            command.linear.y = args.vy
            command.angular.z = args.wz
        values = (command.linear.x, command.linear.y, command.angular.z)
        if not all(math.isfinite(value) for value in values):
            raise RuntimeError("non-finite command")
        released = args.release_at >= 0.0 and elapsed >= args.release_at
        mode_pub.publish(String(data="TAKEOFF" if released else "GROUND"))
        armed_pub.publish(Bool(data=released))
        disarmed_pub.publish(Bool(data=not released))
        cmd_pub.publish(command)
        rate.sleep()

    stop_until = time.monotonic() + 0.25
    released = args.release_at >= 0.0 and args.duration >= args.release_at
    while time.monotonic() < stop_until and not rospy.is_shutdown():
        cmd_pub.publish(zero)
        mode_pub.publish(String(data="TAKEOFF" if released else "GROUND"))
        armed_pub.publish(Bool(data=released))
        disarmed_pub.publish(Bool(data=not released))
        rospy.sleep(0.01)


if __name__ == "__main__":
    try:
        main()
    except (rospy.ROSInterruptException, rospy.ROSException):
        pass
