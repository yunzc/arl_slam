#!/usr/bin/env python

# Yun Chang 2017 
# Pixhawk provides odom, but unfortunately does not broadcast transforms for the odom 
# so this is the script to do that 

import rospy 
import roslib 
import tf 
from nav_msgs.msg import Odometry

pos = None 
quat = None 

rospy.init_node('mavros_tf', anonymous=True)
br = tf.TransformBroadcaster()

def callBack(data):
	position = data.pose.pose.position 
	orientation = data.pose.pose.orientation
	global pos
	global quat 
	pos = [position.x, position.y, position.z]
	quat = [orientation.x, orientation.y, orientation.z, orientation.w]

rospy.Subscriber('/mavros/local_position/odom', Odometry, callBack)

rate = rospy.Rate(30)
while not rospy.is_shutdown():
	if pos!= None and quat != None:
		br.sendTransform(pos, quat, rospy.Time.now(), '/camera_link', '/odom')
		br.sendTransform([0,0,0], [0,0,0,1], rospy.Time.now(), '/odom', '/map')


