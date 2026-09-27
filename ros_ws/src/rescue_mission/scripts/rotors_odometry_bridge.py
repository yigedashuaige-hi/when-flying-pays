#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry


class RotorsOdometryBridge:
    """Expose RotorS odometry on the legacy /odom contract without rewriting it."""

    def __init__(self):
        source = rospy.get_param("~source", "/uav_v4/odometry")
        target = rospy.get_param("~target", "/odom")
        self.publisher = rospy.Publisher(target, Odometry, queue_size=10)
        rospy.Subscriber(source, Odometry, self.callback, queue_size=10)

    def callback(self, msg):
        self.publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node("rotors_odometry_bridge")
    RotorsOdometryBridge()
    rospy.spin()
