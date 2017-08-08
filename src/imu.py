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
		self imu_topic_name = imu_topic_name
		self.accel = []
		# accel in list [a_x, a_y, a_z]
		self.gyro = [] 
		# gyro values in list [w_x, w_y, w_z]
		# ^^ use list as to store not only current data but also data before 
		# according to the size of the filter 
		self.filter_size = median_filter_size
		# initialize ros node 
		rospy.init_node('kf', anonymous=True)
		# define publihser 
		self.pub = rospy.Publisher(imu_state, Imu, queue_size=10)
		# for graphing 
		self.past_accel_data = [[0,0,0] for i in range(50)]
		self.past_gyro_data = [[0,0,0] for j in range(50)]
		self.fig = plt.figure()
		self.ax1 = self.fig.add_subplot(1,1,1)

	def update_data(self):
		# first discard old data 
		if len(self.prev5accel) == self.filter_size:
			self.prev5accel.pop(0)
		if len(self.prev5gyro) == self.filter_size:
			self.prev5gyro.pop(0)
		# add prev data to list
		self.prev5accel.append(self.accel)
		self.prev5gyro.append(self.gyro)

	def IMU_callback_1(self):
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

	def getIMU_1(self):
		self.update_data()
		rospy.Subscriber(self.imu_topic_name[0], Imu, self.IMU_callback_1)

	def IMU_callback_2_accel(self):
		# this is for getIMU_2 for acceleration
		# get new data 
		# update 

	def IMU_callback_2_gyro(self):
		# this is for getIMU_2 for gyro 
		# get new data 
		# update

	def get_IMU_2(self):
		self.update_data()
		rospy.Subscriber(self.imu_topic_name[0], Imu, self.IMU_callback_2_accel)
		rospy.Subscriber(self.imu_topic_name[1], Imu, self.IMU_callback_2_gyro)

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
		mid_index = int(round(self.filter_size/2.))
		accel = [sorted_accel_x[mid_index], sorted_accel_y[mid_index], sorted_accel_z[mid_index]]
		gyro = [sorted_gyro_x[mid_index], sorted_gyro_y[mid_index], sorted_gyro_z[mid_index]]
		return accel, gyro

	def update_plot(self):
		if self.imu_topic_type == 0:
				self.getIMU_1()
		else:
			self.getIMU_2()
		accel, gyro = self.filter_data()
		self.past_accel_data.pop(0)
		self.past_accel_data.append(accel)
		self.past_gyro_data.pop(0)
		self.past_accel_data.append(gyro)
		self.ax1.clear()
		self.ax1.plot([self.past_accel_data[i][0] for i in range(len(self.past_accel_data))])

	def plot_imu_data(self):
		ani = animation.FuncAnimcation(self.fig, update_plot, interval=1000)

if __name__ == '__main__':
	
			






