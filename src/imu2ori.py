#!/usr/bin/env python

# Yun Chang 2017 
# Have to write this file since for some reason the realsense published imu message doesn't come with the
# orientation (maybe they got lazy?) I have to write a script to do so 

import rospy
import roslib 
import numpy as np 
from sensor_msgs.msg import Imu 
from geometry_msgs.msg import Quaternion, Vector3
import tf 

# complimentary filter param 
gyro_weight = 0.98
accel_weight = 0.02 

# covariance value 
covar = 0.01 
cov = [0 for i in range(9)]
cov[0] = covar; ori_cov[4] = covar; ori_cov[8] = covar 

imu_msg = None 
# define callBack function 
def callBack(data):
	# get data 
	accel = data.linear_acceleration
	omeg = data.angular_velocity 
	# change axis (look up realsense doc)#
	# realsens z points forward 
	# realsense x points left 
	# realsense y points up 
	a = [accel.z, accel.x, accel.y]
	w = [omeg.z, omeg.x, omeg.y]
	# calculate angle from acceleration 
	# rotation around x axis results in the downward pointing 
	# vector to move in y dir 
	roll_ang = np.arctan2(-a[1], a[2])
	# comp filter 
	roll = w[0]*gyro_weight + roll_ang*accel_weight
	# likewise around y axis 
	pitch_ang = np.arctan2(a[0], a[2])
	pitch = w[1]*gyro_weight + pitch_ang*accel_weight
	yaw = w[2] 
	msg = Imu()
	msg.linear_acceleration = Vector3(a[0], a[1], a[2])
	msg.angular_velocity = Vector3(w[0], w[1], w[2])
	msg.linear_acceleration_covariance = tuple(cov)
	msg.angular_velocity_covariance = tuple(cov)
	quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
	msg.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
	
	msg.orientation_covariance = tuple(cov)
	global imu_msg 
	imu_msg = msg 

# initialize node, publisher, subscriber 
# will publish as another imu msg this time with orientation
rospy.init_node('imu2ori', anonymous=True)
rospy.Subscriber('/camera/imu/data_raw', Imu, callBack)
pub = rospy.Publisher('/camera/imu/data', Imu, queue_size=10)

# rate 
r = rospy.Rate(30)
# publish 
while not rospy.is_shutdown():
	if imu_msg != None: 
		pub.publish(imu_msg)
	r.sleep()