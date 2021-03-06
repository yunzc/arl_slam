#!/usr/bin/env python

import roslib 
import rospy 
import math
import tf 
import matplotlib.pyplot as plt 

#### need to fix "The messages are in the /camera_link frame (x-axis right, y-axis down and z-axis forward)""
#### so to transform to vicon frame x = z, y = -x, z = -y
#### also need to transform angle roll = yaw, pitch = -roll, yaw = -pitch
def changeSlamAxis(slam_pos, slam_rpy):
	# note do this after already getting the euler form quaternion 
	newpos = [slam_pos[2], -slam_pos[0], -slam_pos[1]]
	newrpy = [slam_rpy[2], -slam_rpy[0], -slam_rpy[1]]
	return newpos, newrpy 

def computeRef(slam_pos, slam_ori, vic_pos, vic_ori):
	# return (x_diff, y_diff, z_diff), (y_diff, p_diff, r_diff)
	slam_rpy = tf.transformations.euler_from_quaternion(slam_ori)
	# s_pos, s_rpy = changeSlamAxis(slam_pos, slam_rpy)
	s_pos, s_rpy = slam_pos, slam_rpy
	vic_rpy = tf.transformations.euler_from_quaternion(vic_ori)
	lin_diff = tuple([vic_pos[i] - s_pos[i] for i in range(len(s_pos))])
	ang_diff = tuple([vic_rpy[j] - s_rpy[j] for j in range(len(s_rpy))])
	return lin_diff, ang_diff

def transformSlamData(slam_pos, slam_ori, lin_diff, ang_diff):
	slam_rpy = tf.transformations.euler_from_quaternion(slam_ori)
	# s_pos, s_rpy = changeSlamAxis(slam_pos, slam_rpy)
	s_pos, s_rpy = slam_pos, slam_rpy
	rpy = [s_rpy[i] + ang_diff[i] for i in range(len(ang_diff))] # transform the slam data to the vicon frame 
	pos = [s_pos[j] + lin_diff[j] for j in range(len(lin_diff))]
	return pos, rpy

def findDifference(new_slam_pos, new_slam_rpy, vic_pos, vic_rpy):
	diff_pos = [new_slam_pos[i] - vic_pos[i] for i in range(len(vic_pos))]
	diff_rpy = [new_slam_rpy[j] - vic_rpy[j] for j in range(len(vic_rpy))]
	print(diff_pos)
	return diff_pos, diff_rpy

def findDistance(diff_pos):
	return math.sqrt(diff_pos[0]**2 + diff_pos[1]**2 + diff_pos[2]**2)

def rads2degs(rad):
	degrees = rad/6.28*360
	return degrees
	
class PlotAndStore(object):
	# store data and create plots
	def __init__(self, sets_of_data, downsample_factor=30):
		# sets of data should be an integer
		self.data = [[] for i in range(sets_of_data)]
		self.n = [0 for i in range(sets_of_data)]
		self.downsample_factor = downsample_factor

	def add_datapt(self, data_index, data_pt):
		if self.n[data_index] % self.downsample_factor == 0:
			self.data[data_index].append(data_pt)
		self.n[data_index] += 1

	def show_result(self, TitlesList, yAxisList):
		for i in range(len(self.data)):
			plt.plot(self.data[i])
			plt.ylabel(yAxisList[i])
			plt.title(TitlesList[i])
			plt.show()
			data_len = len(self.data[i])
			print(TitlesList[i])
			print('max value: {}'.format(str(max(self.data[i]))))
			avg = sum(self.data[i])/data_len
			print('avg value: {}'.format(str(avg)))
			avg_1 = sum(self.data[i][0:int(data_len/2)])/float(int((data_len/2)))
			avg_2 = sum(self.data[i][int(data_len/2):])/float(int((data_len/2)))
			print('first half avg value: {}'.format(str(avg_1)))
			print('second half avg value: {}'.format(str(avg_2))) 

if __name__ == '__main__':
	vicon_pos_0 = None # xyz from vicon 
	vicon_ori_0 = None # rpy from vicon 

	slam_pos_0 = None # xyz from rtabmap slam 
	slam_ori_0 = None # rpy from rtabmap slam 

	rospy.init_node('analyzer')
	listener = tf.TransformListener()
	rate = rospy.Rate(30.0)

	lin_diff = None # difference in the linear position 
	ang_diff = None # difference in the angular position 

	analyzer = PlotAndStore(6)

	while not rospy.is_shutdown():
		try:
			(trans_v,rot_v) = listener.lookupTransform('vicon/yun/yun', '/vicon_origin', rospy.Time(0))
			(trans_s, rot_s) = listener.lookupTransform('base_link', 'odom', rospy.Time(0))
			if vicon_pos_0 == None or vicon_ori_0 == None: # find the first transform for reference 
				vicon_pos_0 = trans_v
				vicon_ori_0 = rot_v

			if slam_pos_0 == None or slam_ori_0 == None: # keep first transform 
				slam_pos_0 = trans_s
				slam_ori_0 = rot_s

			if lin_diff == None or ang_diff == None: 
				if slam_pos_0 != None and slam_ori_0 != None and vicon_ori_0 != None and vicon_pos_0 != None:
					# compute reference 
					lin_diff, ang_diff = computeRef(slam_pos_0, slam_ori_0, vicon_pos_0, vicon_ori_0)
					print("reading...")
			else:
				rot_rpy = tf.transformations.euler_from_quaternion(rot_v) 
				pos, rpy = transformSlamData(trans_s, rot_s, lin_diff, ang_diff)
				diff_pos, diff_rpy = findDifference(pos, rpy, trans_v, rot_rpy)
				dist = findDistance(diff_pos)
				analyzer.add_datapt(0, abs(diff_pos[0]))
				analyzer.add_datapt(1, abs(diff_pos[1]))
				analyzer.add_datapt(2, abs(diff_pos[2]))
				analyzer.add_datapt(3, abs(diff_rpy[0]))
				analyzer.add_datapt(4, abs(diff_rpy[1]))
				analyzer.add_datapt(5, abs(diff_rpy[2]))

		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue
		rate.sleep()
	analyzer.show_result(['x-dist', 'y-dist', 'z-dist', 'roll', 'pitch', 'yaw'], ['m', 'm', 'm','rads', 'rads', 'rads'])
