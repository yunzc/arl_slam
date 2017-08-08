#!/usr/bin/env python  
import roslib
import rospy
import numpy as np 
from sensor_msgs.msg import Imu 
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Point, Quaternion, Vector3
import matplotlib.pyplot as plt 
import matplotlib.animation as animation 
import tf
import threading 

class IMU_state_estimator(object):
	# this class will take IMU data and convert 
	# it into a state estimate 
	def __init__(self, imu_topic_name, median_filter_size=999):
		# imu topic name could be: (must be put in list) 
		# ['imu/data_raw']
		# or [accel/sample, gyro/sample]
		self.imu_topic_type = 0	
		if len(imu_topic_name) == 2:
			self.imu_topic_type = 1
		self.imu_topic_name = imu_topic_name
		self.accel = []
		# accel in list [a_x, a_y, a_z]
		self.gyro = [] 
		# gyro values in list [w_x, w_y, w_z]
		# ^^ use list as to store not only current data but also data before 
		# according to the size of the filter 
		self.filter_size = median_filter_size
		# initialize ros node 
		rospy.init_node('state_estimator', anonymous=True)
		# define publihser 
		self.pub = rospy.Publisher('imu_odom', Odometry, queue_size=10)
		# for graphing 
		self.past_accel_data = [[0,0,0] for i in range(50)]
		self.past_gyro_data = [[0,0,0] for j in range(50)]
		# pose estimation 
		self.position = [0,0,0] # start off at origin 
		self.velocity = [0,0,0]
		self.orientation = [0,0,0] # row pitch yaw 
		self.past_position_data = [[0,0,0] for i in range(50)]
		self.past_orientation_data = [[0,0,0] for j in range(50)]
		# store initial value for calibration 
		self.accel_offset = None
		self.gyro_offset = None 
		# locks to prevent race conditions 
		self.lock = threading.Lock()

	def update_data(self):
		# discard old data 
		if len(self.accel) >= self.filter_size:
			a_num = len(self.accel) - self.filter_size
			self.accel = self.accel[a_num:]
		if len(self.gyro) >= self.filter_size:
			g_num = len(self.gyro) - self.filter_size
			self.gyro = self.gyro[g_num:]

	def callBack(self, data):
		# this is for getIMU_1 
		# get new data 
		linAcc = data.linear_acceleration
		angVel = data.angular_velocity 
		# update 
		# for the realsense, z is in direction of viewing (front of cam)
		# y is down, x is to the right 
		# I want it to be instead: z up. x forward, y to the left 
		self.accel.append([linAcc.z, -linAcc.x, -linAcc.y])
		self.gyro.append([angVel.z, -angVel.x, -angVel.y])
		if self.accel_offset == None:
			self.accel_offset = self.accel[-1]
		if self.gyro_offset == None:
			self.gyro_offset = self.gyro[-1]

	def getIMU1(self):
		self.update_data()
		rospy.Subscriber(self.imu_topic_name[0], Imu, self.callBack)

	def callBackAccel(self, data):
		# this is for getIMU_2 for acceleration
		# get new data 
		linAcc = data.linear_acceleration
		# update 
		self.accel.append([linAcc.z, -linAcc.x, -linAcc.y])
		if self.accel_offset == None:
			self.accel_offset = self.accel[-1]

	def callBackGyro(self, data):
		# this is for getIMU_2 for gyro 
		# get new data 
		angVel = data.angular_velocity 
		# update
		self.gyro.append([angVel.z, -angVel.x, -angVel.y])
		if self.gyro_offset == None:
			self.gyro_offset = self.gyro[-1]

	def getIMU2(self):
		self.update_data()
		rospy.Subscriber(self.imu_topic_name[0], Imu, self.callBackAccel)
		rospy.Subscriber(self.imu_topic_name[1], Imu, self.callBackGyro)

	def filter_data(self):
		# median filter
		# sort the values in preparation to take median 
		# prevent race conditions in call back 
		self.lock.acquire()
		sorted_accel_x = sorted([self.accel[i][0] for i in range(len(self.accel))])
		sorted_accel_y = sorted([self.accel[i][1] for i in range(len(self.accel))])
		sorted_accel_z = sorted([self.accel[i][2] for i in range(len(self.accel))])
		sorted_gyro_x = sorted([self.gyro[j][0] for j in range(len(self.gyro))])
		sorted_gyro_y = sorted([self.gyro[j][1] for j in range(len(self.gyro))])
		sorted_gyro_z = sorted([self.gyro[j][2] for j in range(len(self.gyro))])
		self.lock.release()
		# take middle value of sorted data 
		mid_index = int(round(len(self.accel)/2.)) - 1
		accel = [sorted_accel_x[mid_index], sorted_accel_y[mid_index], sorted_accel_z[mid_index]]
		gyro = [sorted_gyro_x[mid_index], sorted_gyro_y[mid_index], sorted_gyro_z[mid_index]]
		return accel, gyro

	def update_compfilt_ref(self, acc_0):
		self.gravity = np.sqrt(acc_0[0]**2 + acc_0[1]**2 + acc_0[2]**2)
		# note that gravity usually points in the negative z dir 
		# so we are gonna calculate how much the offset is as calibration
		ayz_vector = np.sqrt(acc_0[1]**2 + acc_0[2]**2)
		pitch_offset = -np.arcos(ayz_vector/self.gravity)*acc_0[0]/abs(acc_0[0])
		roll_offset = 0 # confused here can't really work it out right now 

	def plot_imu_data(self):
		rate = rospy.Rate(30)
		while not rospy.is_shutdown():
			if self.imu_topic_type == 0:
				self.getIMU1()
			else:
				self.getIMU2()
			if len(self.accel) != 0 and len(self.gyro) != 0:
				accel, gyro = self.filter_data() # [self.accel[-1], self.gyro[-1]]
				self.past_accel_data.pop(0)
				self.past_accel_data.append(accel)
				self.past_gyro_data.pop(0)
				self.past_gyro_data.append(gyro)
				plt.plot([self.past_accel_data[i][0] for i in range(len(self.past_accel_data))])
				plt.pause(1/30.)
				plt.clf()
			rate.sleep()
		plt.show()

	def update_pos(self, dt):
		acc, omeg = self.filter_data()
		# subtract offset 
		acc = [acc[i] - self.accel_offset[i] for i in range(len(acc))]
		omeg = [omeg[i] - self.gyro_offset[i] for i in range(len(omeg))]
		vel = [self.velocity[i] + acc[i]*dt for i in range(3)]
		pos = [self.position[i] + vel[i]*dt for i in range(3)]
		ori = [self.orientation[i] + omeg[i]*dt for i in range(3)]
		self.position = pos
		self.velocity = vel
		self.orientation = ori

	def plot_pose(self):
		rate = rospy.Rate(30)
		while not rospy.is_shutdown():
			if self.imu_topic_type == 0:
				self.getIMU1()
			else:
				self.getIMU2()
			if len(self.accel) != 0 and len(self.gyro) != 0:
				self.update_pos(1/30.)
				self.past_position_data.pop(0)
				self.past_position_data.append(self.position)
				self.past_orientation_data.pop(0)
				self.past_orientation_data.append(self.orientation)
				plt.plot([self.past_orientation_data[i][0] for i in range(len(self.past_position_data))])
				plt.pause(1/30.)
				plt.clf()
			rate.sleep()
		plt.show()

	def publishOdom(self):
		rate = rospy.Rate(30)
		while not rospy.is_shutdown():
			if self.imu_topic_type == 0:
				self.getIMU1()
			else:
				self.getIMU2()
			if len(self.accel) != 0 and len(self.gyro) != 0:
				self.update_pos(1/30.)
				# publish odom topic 
				msg = Odometry()
				msg.header.stamp = rospy.Time.now()
				msg.header.frame_id = '/odom' # i.e. '/odom'
				msg.child_frame_id = '/camera_link' # i.e. '/base_link'

				msg.pose.pose.position = Point(self.position[0], self.position[1], self.position[2])
				quat = tf.transformations.quaternion_from_euler(self.orientation[0], self.orientation[1], self.orientation[2])
				msg.pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
				msg.pose.covariance = tuple([0 for i in range(36)])
				msg.twist.twist.linear = Vector3(self.position[0], self.position[1], self.position[2])
				msg.twist.twist.angular = Vector3(self.orientation[0], self.orientation[1], self.orientation[2])
				msg.twist.covariance = tuple([0 for i in range(36)])
				self.pub.publish(msg)
			rate.sleep()

if __name__ == '__main__':
	test = IMU_state_estimator(['/camera/accel/sample', '/camera/gyro/sample'])
	test.publishOdom()
	

