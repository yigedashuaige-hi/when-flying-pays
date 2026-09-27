#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class AirModeStub:
    def __init__(self):
        self.mode = "GROUND"
        self.rate_hz = rospy.get_param("~update_rate", 10.0)
        self.cmd_pub = rospy.Publisher("/air_stub/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/rescue/mode", String, self.mode_cb, queue_size=1)

    def mode_cb(self, msg):
        self.mode = msg.data

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            cmd = Twist()
            if self.mode == "TAKEOFF":
                cmd.linear.z = 0.4
            elif self.mode == "AIR":
                cmd.linear.z = 0.0
            elif self.mode == "LAND":
                cmd.linear.z = -0.3
            self.cmd_pub.publish(cmd)
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("air_mode_stub")
    AirModeStub().spin()
