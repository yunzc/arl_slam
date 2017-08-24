#!/usr/bin/env python

# Yun Chang 2017 
# Realsense frame is given in a z-backward, x- left, y-up form (left-handed)
# Pixhawk frame is given in a z-down, x-backward, y-right form (left-handed)
# Rtabmap; however, is normal and does an x-forward, y-left, z-up form 

import rospy
import roslib 
from sensor_msgs.msg import Imu 
import tf 
from geometry_msgs.msg import Point, Quaternion, Vector3

def convertRSframe(accel, gyro, ori, accel_cov, gyro_cov, ori_cov):
	# ori (orientation) quaternion 
	newaccel = Vector3(-accel.z, accel.x, accel.y)
	newgyro = Vector3(-gyro.z, gyro.x, gyro.y)
	eul = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w]) 
	newquat = tf.transformations.quaternion_from_euler(-eul[2], eul[0], eul[1])
	newori = Quaternion(newquat[0], newquat[1], newquat[2], newquat[3])
	return [newaccel, newgyro, newori, accel_cov, gyro_cov, ori_cov]

def convertPXframe(accel, gyro, ori, accel_cov, gyro_cov, ori_cov):
	newaccel = Vector3(-accel.x, -accel.y, -accel.z)
	newgyro = Vector3(-gyro.x, -gyro.y, -gyro.z)
	eul = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w]) 
	newquat = tf.transformations.quaternion_from_euler(-eul[0], -eul[1], -eul[2])
	newori = Quaternion(newquat[0], newquat[1], newquat[2], newquat[3])
	return [newaccel, newgyro, newori, accel_cov, gyro_cov, ori_cov]

def callBackRS(data):
	g = data.angular_velocity 
	g_cov = data.angular_velocity_covariance
	a = data.linear_acceleration
	a_cov = data.linear_acceleration_covariance
	o = data.orientation
	o_cov = data.orientation_covariance
	global RSimu
	RSimu = convertRSframe(a, g, o, a_cov, g_cov, o_cov)

def callBackPX(data):
	g = data.angular_velocity 
	g_cov = data.angular_velocity_covariance
	a = data.linear_acceleration
	a_cov = data.linear_acceleration_covariance
	o = data.orientation
	o_cov = data.orientation_covariance
	global PXimu
	PXimu = convertPXframe(a, g, o, a_cov, g_cov, o_cov)

RSimu = None 
PXimu = None

rospy.init_node('frameConv', anonymous=True)
PXpub = rospy.Publisher('/mavros/imu/data_adjusted', Imu, queue_size=10)
RSpub = rospy.Publisher('/camera/imu/data_adjusted', Imu, queue_size=10)

rospy.Subscriber('/mavros/imu/data', Imu, callBackPX)
rospy.Subscriber('/camera/imu/data_raw', Imu, callBackRS)

rate = rospy.Rate(30)
while not rospy.is_shutdown():
	if PXimu != None:
		PXmsg = Imu()
		PXmsg.linear_acceleration = PXimu[0]
		PXmsg.angular_velocity = PXimu[1]
		PXmsg.orientation = PXimu[2]
		PXmsg.linear_acceleration_covariance = PXimu[3]
		PXmsg.angular_velocity_covariance = PXimu[4]
		PXmsg.orientation_covariance = PXimu[5]
		PXpub.publish(PXmsg)
	if RSimu != None: 
		RSmsg = Imu()
		RSmsg.linear_acceleration = RSimu[0]
		RSmsg.angular_velocity = RSimu[1]
		RSmsg.orientation = RSimu[2]
		RSmsg.linear_acceleration_covariance = RSimu[3]
		RSmsg.angular_velocity_covariance = RSimu[4]
		RSmsg.orientation_covariance = RSimu[5]
		RSpub.publish(RSmsg)
	rate.sleep()

