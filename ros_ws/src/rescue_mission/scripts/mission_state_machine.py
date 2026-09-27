#!/usr/bin/env python3
import math

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


class MissionStateMachine:
    STATES = ("SEARCH", "DETECT", "NAVIGATE", "DELIVER", "RETURN", "DONE")

    def __init__(self):
        self.rate_hz = rospy.get_param("~update_rate", 2.0)
        self.goal_tolerance = rospy.get_param("~goal_tolerance", 0.5)
        self.waypoints = rospy.get_param("~waypoints", [[2.0, 0.0, 0.0]])
        initial_state = rospy.get_param("~initial_state", "NAVIGATE")

        self.state_index = self.STATES.index(initial_state) if initial_state in self.STATES else 2
        self.waypoint_index = 0
        self.odom = None
        self.mode = "GROUND"

        self.state_pub = rospy.Publisher("/rescue/mission_state", String, queue_size=10, latch=True)
        self.goal_pub = rospy.Publisher("/rescue/goal", PoseStamped, queue_size=10, latch=True)
        rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/rescue/mode", String, self.mode_cb, queue_size=1)

    def odom_cb(self, msg):
        self.odom = msg

    def mode_cb(self, msg):
        self.mode = msg.data

    def current_state(self):
        return self.STATES[min(self.state_index, len(self.STATES) - 1)]

    def current_goal(self):
        wp = self.waypoints[min(self.waypoint_index, len(self.waypoints) - 1)]
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "odom"
        goal.pose.position.x = wp[0]
        goal.pose.position.y = wp[1]
        goal.pose.position.z = wp[2] if len(wp) > 2 else 0.0
        goal.pose.orientation.w = 1.0
        return goal

    def reached_goal(self):
        if self.odom is None:
            return False
        goal = self.current_goal()
        px = self.odom.pose.pose.position.x
        py = self.odom.pose.pose.position.y
        gx = goal.pose.position.x
        gy = goal.pose.position.y
        return math.hypot(gx - px, gy - py) < self.goal_tolerance

    def step(self):
        state = self.current_state()
        if state == "SEARCH" and self.mode == "AIR":
            self.state_index = 1
        elif state == "DETECT":
            self.state_index = 2
        elif state == "NAVIGATE" and self.reached_goal():
            self.waypoint_index += 1
            if self.waypoint_index >= len(self.waypoints):
                self.state_index = 3
        elif state == "DELIVER" and self.mode == "GROUND":
            self.state_index = 4
            self.waypoint_index = 0
        elif state == "RETURN" and self.reached_goal():
            self.state_index = 5

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self.step()
            self.goal_pub.publish(self.current_goal())
            self.state_pub.publish(String(data=self.current_state()))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("mission_state_machine")
    MissionStateMachine().spin()
