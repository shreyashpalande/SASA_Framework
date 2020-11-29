#!/usr/bin/env python  

import roslib
import rospy
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('posepublisher')

    listener = tf.TransformListener()
    turtle_vel = rospy.Publisher('cartopose', geometry_msgs.msg.PoseWithCovarianceStamped,queue_size=1)
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/wheel_right_link', '/chassis_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        carto=geometry_msgs.msg.PoseWithCovarianceStamped()
        carto.header.stamp=rospy.Time.now()
        # carto.header.frame_id="map" #use when no robot localization node
        # carto.header.frame_id="map"


        # carto.pose.pose.position.x=trans[0]
        # carto.pose.pose.position.y=trans[1]
        # carto.pose.pose.position.z=trans[2]

        # carto.pose.pose.orientation.x=rot[0]
        # carto.pose.pose.orientation.y=rot[1]
        # carto.pose.pose.orientation.z=rot[2]
        # carto.pose.pose.orientation.w=rot[3]

        # turtle_vel.publish(carto)
        print(rot)

        rate.sleep()