#!/usr/bin/env python3
import math

import rospy
from mav_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String


class RotorsMotorGate:
    """Pass Lee outputs only while airborne and latch zero motors at touchdown."""

    def __init__(self):
        self.motor_count = int(rospy.get_param("~motor_count", 4))
        self.touchdown_height = float(rospy.get_param("~touchdown_height", 0.09))
        self.touchdown_vertical_speed = float(
            rospy.get_param("~touchdown_vertical_speed", 0.12)
        )
        self.touchdown_horizontal_speed = float(
            rospy.get_param("~touchdown_horizontal_speed", 0.15)
        )
        self.touchdown_hold_sec = float(rospy.get_param("~touchdown_hold_sec", 0.5))
        self.publish_rate = float(rospy.get_param("~publish_rate", 100.0))
        self.max_motor_speed = float(rospy.get_param("~max_motor_speed", 1100.0))
        self.handoff_managed = bool(rospy.get_param("~handoff_managed", False))

        raw_topic = rospy.get_param(
            "~raw_motor_topic", "/uav_v4/command/motor_speed_raw"
        )
        output_topic = rospy.get_param(
            "~motor_topic", "/uav_v4/command/motor_speed"
        )
        odometry_topic = rospy.get_param("~odometry_topic", "/uav_v4/odometry")

        self.mode = "GROUND"
        self.armed = False
        self.odom = None
        self.raw_command = None
        self.touchdown_since = None

        self.command_pub = rospy.Publisher(output_topic, Actuators, queue_size=1)
        self.armed_pub = rospy.Publisher(
            "/rescue/rotors/armed", Bool, queue_size=1, latch=True
        )
        self.disarmed_pub = rospy.Publisher(
            "/rescue/rotors/disarmed", Bool, queue_size=1, latch=True
        )
        self.state_pub = rospy.Publisher(
            "/rescue/rotors/flight_state", String, queue_size=1, latch=True
        )
        rospy.Subscriber("/rescue/mode", String, self.mode_cb, queue_size=1)
        rospy.Subscriber(raw_topic, Actuators, self.raw_cb, queue_size=1)
        rospy.Subscriber(odometry_topic, Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber(
            "/rescue/rotors/motor_enable", Bool, self.motor_enable_cb, queue_size=1
        )

        self.publish_status("GROUND_DISARMED")

    def mode_cb(self, msg):
        previous = self.mode
        self.mode = msg.data
        if self.handoff_managed:
            return
        if self.mode in ("TAKEOFF", "AIR"):
            if not self.armed:
                rospy.loginfo("RotorS motor gate armed for %s", self.mode)
            self.armed = True
            self.touchdown_since = None
        elif self.mode == "GROUND":
            self.armed = False
            self.touchdown_since = None
        elif self.mode == "LAND" and previous != "LAND":
            self.touchdown_since = None

    def motor_enable_cb(self, msg):
        if not self.handoff_managed:
            return
        enable = bool(msg.data)
        if enable and not self.armed:
            rospy.loginfo("RotorS motor gate armed by handoff coordinator")
        if not enable and self.armed:
            rospy.loginfo("RotorS motor gate disarmed by handoff coordinator")
        self.armed = enable
        if not enable:
            self.touchdown_since = None

    def raw_cb(self, msg):
        self.raw_command = msg

    def odom_cb(self, msg):
        self.odom = msg

    def touchdown_observed(self):
        if self.odom is None:
            return False
        position = self.odom.pose.pose.position
        velocity = self.odom.twist.twist.linear
        horizontal_speed = math.hypot(velocity.x, velocity.y)
        return (
            position.z <= self.touchdown_height
            and abs(velocity.z) <= self.touchdown_vertical_speed
            and horizontal_speed <= self.touchdown_horizontal_speed
        )

    def update_touchdown_latch(self):
        if self.handoff_managed:
            return
        if self.mode != "LAND" or not self.armed:
            return
        if not self.touchdown_observed():
            self.touchdown_since = None
            return
        now = rospy.Time.now()
        if self.touchdown_since is None:
            self.touchdown_since = now
            return
        if (now - self.touchdown_since).to_sec() >= self.touchdown_hold_sec:
            self.armed = False
            rospy.loginfo("RotorS touchdown confirmed; motor output latched to zero")

    def zero_command(self):
        msg = Actuators()
        msg.header.stamp = rospy.Time.now()
        msg.angular_velocities = [0.0] * self.motor_count
        return msg

    def safe_raw_command(self):
        if self.raw_command is None:
            return self.zero_command()
        if len(self.raw_command.angular_velocities) != self.motor_count:
            rospy.logwarn_throttle(
                2.0,
                "Ignoring Lee motor vector of length %d; expected %d",
                len(self.raw_command.angular_velocities),
                self.motor_count,
            )
            return self.zero_command()
        values = self.raw_command.angular_velocities
        if not all(math.isfinite(value) for value in values):
            rospy.logerr_throttle(1.0, "Rejecting non-finite Lee motor command")
            return self.zero_command()
        msg = Actuators()
        msg.header = self.raw_command.header
        msg.angular_velocities = [
            min(self.max_motor_speed, max(0.0, value)) for value in values
        ]
        return msg

    def publish_status(self, state=None):
        if state is None:
            if self.armed:
                state = self.mode + "_ARMED"
            elif self.mode == "LAND":
                state = "TOUCHDOWN_DISARMED"
            else:
                state = "GROUND_DISARMED"
        self.armed_pub.publish(Bool(data=self.armed))
        self.disarmed_pub.publish(Bool(data=not self.armed))
        self.state_pub.publish(String(data=state))

    def spin(self):
        rate = rospy.Rate(self.publish_rate)
        while not rospy.is_shutdown():
            self.update_touchdown_latch()
            self.command_pub.publish(
                self.safe_raw_command() if self.armed else self.zero_command()
            )
            self.publish_status()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rotors_motor_gate")
    try:
        RotorsMotorGate().spin()
    except rospy.ROSInterruptException:
        pass
