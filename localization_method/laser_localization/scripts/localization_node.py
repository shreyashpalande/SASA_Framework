#!/usr/bin/env python
from __future__ import print_function
import rospy
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import Joy
import sys
import json
from collections import deque

import time


def callback(data):
        global xAnt
        global yAnt
        global cont

        pose = PoseStamped()

        pose.header.frame_id = "main"
        pose.pose.position.x = float(data.pose.pose.position.x)
        pose.pose.position.y = float(data.pose.pose.position.y)
        pose.pose.position.z = float(data.pose.pose.position.z)
        pose.pose.orientation.x = float(data.pose.pose.orientation.x)
        pose.pose.orientation.y = float(data.pose.pose.orientation.y)
        pose.pose.orientation.z = float(data.pose.pose.orientation.z)
        pose.pose.orientation.w = float(data.pose.pose.orientation.w)

        if (xAnt != pose.pose.position.x and yAnt != pose.pose.position.y):
                pose.header.seq = path.header.seq + 1
                path.header.frame_id = "main"
                path.header.stamp = rospy.Time.now()
                pose.header.stamp = path.header.stamp
                path.poses.append(pose)
                # Published the msg

        cont = cont + 1

        rospy.loginfo("Hit: %i" % cont)
        if cont > max_append:
                path.poses.pop(0)

        pub.publish(path)

        xAnt = pose.pose.orientation.x
        yAnt = pose.pose.position.y
        return path


if __name__ == '__main__':
        # Initializing node
        rospy.init_node('path_plotter')

        pub = rospy.Publisher('/laser_odom', Odometry, queue_size=1)

        msg = Odometry()

        # Subscription to the required odom topic (edit accordingly)
        msg = rospy.Subscriber('/odom_rf2o', Odometry, callback)

        rate = rospy.Rate(30)  # 30hz

        try:
                while not rospy.is_shutdown():
                    # rospy.spin()
                    rate.sleep()
        except rospy.ROSInterruptException:
                pass