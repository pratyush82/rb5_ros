#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose2D
import time

if __name__ == "__main__":
    rospy.init_node("hw1_pub")
    pub_joy = rospy.Publisher("/hw1", Pose2D, queue_size=5)
    points = []
    with open("src/rb5_ros/hw1/src/waypoints.txt") as file:
        lines = file.readlines()
        points = [line.rstrip() for line in lines]
    for p in points:
        p = p.split(',')
        msg = Pose2D(x=float(p[0]), y=float(p[1]), theta=float(p[2]))
        pub_joy.publish(msg)
        time.sleep(5)
