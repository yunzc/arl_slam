#!/usr/bin/env python  
import roslib
import rospy
import numpy as np 
from sensor_msgs.msg import Imu 
import matplotlib.pyplot as plt 
import matplotlib.animation as animation 

class IMU_state_estimator(object):
	# this class will take IMU data and convert 
	# it into a state estimate 
	def __init__(self, imu_topic_name, median_filter_size=5):
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
		self.pub = rospy.Publisher('imu_state', Imu, queue_size=10)
		# for graphing 
		self.past_accel_data = [[0,0,0] for i in range(50)]
		self.past_gyro_data = [[0,0,0] for j in range(50)]

	def update_data(self):
		# discard old data 
		if len(self.accel) == self.filter_size:
			self.accel.pop(0)
		if len(self.gyro) == self.filter_size:
			self.gyro.pop(0)

	def callBack(self, data):
		# this is for getIMU_1 
		# get new data 
		linAcc = data.linear_acceleration
		linCov = np.matrix(data.linear_acceleration_covariance)
		linCov = linCov.reshape([3,3])
		angVel = data.angular_velocity 
		angCov = np.matrix(data.angular_velocity_covariance)
		angCov = angCov.reshape([3,3])
		# update 
		self.accel = [linAcc.x, linAcc.y, linAcc.z]
		self.gyro = [angVel.x,angVel.y,angVel.z]

	def getIMU1(self):
		self.update_data()
		rospy.Subscriber(self.imu_topic_name[0], Imu, self.callBack)

	def callBackAccel(self, data):
		# this is for getIMU_2 for acceleration
		# get new data 
		linAcc = data.linear_acceleration
		linCov = np.matrix(data.linear_acceleration_covariance)
		linCov = linCov.reshape([3,3])
		# update 
		self.accel.append([linAcc.x, linAcc.y, linAcc.z])

	def callBackGyro(self, data):
		# this is for getIMU_2 for gyro 
		# get new data 
		angVel = data.angular_velocity 
		angCov = np.matrix(data.angular_velocity_covariance)
		angCov = angCov.reshape([3,3])
		# update
		self.gyro.append([angVel.x,angVel.y,angVel.z])

	def getIMU2(self):
		self.update_data()
		rospy.Subscriber(self.imu_topic_name[0], Imu, self.callBackAccel)
		rospy.Subscriber(self.imu_topic_name[1], Imu, self.callBackGyro)

	def filter_data(self):
		# median filter
		# sort the values in preparation to take median 
		sorted_accel_x = sorted([self.accel[i][0] for i in range(len(self.accel))])
		sorted_accel_y = sorted([self.accel[i][1] for i in range(len(self.accel))])
		sorted_accel_z = sorted([self.accel[i][2] for i in range(len(self.accel))])
		sorted_gyro_x = sorted([self.gyro[j][0] for j in range(len(self.gyro))])
		sorted_gyro_y = sorted([self.gyro[j][1] for j in range(len(self.gyro))])
		sorted_gyro_z = sorted([self.gyro[j][2] for j in range(len(self.gyro))])
		# take middle value of sorted data 
		mid_index = int(round(len(self.accel)/2.)) - 1
		accel = [sorted_accel_x[mid_index], sorted_accel_y[mid_index], sorted_accel_z[mid_index]]
		gyro = [sorted_gyro_x[mid_index], sorted_gyro_y[mid_index], sorted_gyro_z[mid_index]]
		return accel, gyro

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
				plt.plot([self.past_accel_data[i][2] for i in range(len(self.past_accel_data))])
				plt.pause(1/30.)
				plt.clf()
			rate.sleep()
		plt.show()


if __name__ == '__main__':
	test = IMU_state_estimator(['/camera/accel/sample', '/camera/gyro/sample'])
	test.plot_imu_data()
	

