#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class AirGoalBridge:
    def __init__(self):
        self.cruise_altitude = rospy.get_param("~cruise_altitude", 1.2)
        self.ground_altitude = rospy.get_param("~ground_altitude", 0.0)
        self.frame_id = rospy.get_param("~frame_id", "world")
        target_topic = rospy.get_param("~target_topic", "/rescue/air_goal")
        command_pose_topic = rospy.get_param("~rotors_command_pose_topic", "/command/pose")
        self.publish_rotors_command_pose = rospy.get_param("~publish_rotors_command_pose", False)

        self.mode = "GROUND"
        self.goal = None
        self.air_goal_pub = rospy.Publisher(target_topic, PoseStamped, queue_size=10)
        self.command_pose_pub = rospy.Publisher(command_pose_topic, PoseStamped, queue_size=10)

        rospy.Subscriber("/rescue/mode", String, self.mode_cb, queue_size=1)
        rospy.Subscriber("/rescue/goal", PoseStamped, self.goal_cb, queue_size=1)

    def mode_cb(self, msg):
        self.mode = msg.data

    def goal_cb(self, msg):
        self.goal = msg

    def target_altitude(self):
        if self.mode in ("TAKEOFF", "AIR"):
            return self.cruise_altitude
        return self.ground_altitude

    def publish(self):
        if self.goal is None:
            return

        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.pose = self.goal.pose
        msg.pose.position.z = self.target_altitude()

        self.air_goal_pub.publish(msg)
        if self.publish_rotors_command_pose and self.mode in ("TAKEOFF", "AIR", "LAND"):
            self.command_pose_pub.publish(msg)

    def spin(self):
        rate = rospy.Rate(rospy.get_param("~update_rate", 20.0))
        while not rospy.is_shutdown():
            self.publish()
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("air_goal_bridge")
    AirGoalBridge().spin()
