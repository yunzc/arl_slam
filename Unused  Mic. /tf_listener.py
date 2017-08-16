#!/usr/bin/env python 

# for debugging purposes 

import roslib
import rospy
import math
import tf

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    rate = rospy.Rate(30.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/camera_link', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        print(trans)

        rate.sleep()
