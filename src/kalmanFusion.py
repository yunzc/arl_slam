#!/usr/bin/env python  
import roslib
import rospy
import numpy as np 
from sensor_msgs.msg import Imu 
from nav_msgs.msg import Odometry 

# define identity matrix 
I = np.matrix(np.identity)

def CallVisOdom(data):
	# return [pose twist] 
	return [data.pose data.twist]

def getVisualOdom(VisOdomTopic="camera/odom"):
	rospy.subscriber(VisOdomTopic, Odometry, CallVisOdom)

def CallImu(data):
	# return [pose covariance]
	return [data.pose data.covariance]

def getImuData(ImuTopic="camera/imu/data_raw"):
	rospy.subscriber(ImuTopic, Imu, CallImu)

def computeEstimateTranslation(U, Q, Z, P, R, xt_prev, d_t):
	# the states i want will be x, xdot, y, ..., zdot
	# and yaw pitch roll with their velocities 
	# but i shall turn the two into independent computations 
	# input previous translational state vector and rotational 
	# state vector 
	F_t = np.matrix([[1, d_t, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, \
		1, d_t, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, d_t], [0, \
		0, 0, 0, 0, 1]]) # transform matrix from previous state 
	B_t = np.matrix([[.5*d_t**2, 0, 0], [d_t, 0, 0], [0, .5*d_t**2, 0], \
		[0, d_t, 0], [0, 0, .5*d_t**2], [0, 0, d_t]]) # mapping from input (in this case accel)
	xt_ = F*xt_prev + B_t*U # first translational estimation 
	H = I # In our case the "measurement" is visual odom 
	P_ = F_t*P*np.matrix.transpose(F_t) + Q
	S = H*P_*np.matrix.transpose(H) + R 
	K = P_*np.matrix.transpose(H)*np.matrix.inverse(S) # Kalman gain 
	y = Z - H*xt_prev
	x_est = xt_ + K*y 
	P_est = (I - K*H)*P_
	return x_est, P_est 

def computeEstimateRotation(U, Q, Z, P, xr_prev, d_t):
	F_r = I
	B_r = d_t*I 
	xr_ = F*xr_prev + B_r*U # first rotational estimation 
	H = I 
	P_ = F_r*P*np.matrix.transpose(F_t) + Q 
	S = H*P_*np.matrix.transpose(H) + R 
	K = P_*np.matrix.transpose(H)*np.matrix.inverse(S)
	y = Z - H*xr_prev
	x_est = xr_ + K*y 
	P_est = (I - K*H)*P_ 
	return x_est, P_est

if __name__ == '__main__':
	# rospy.init_node('kf', anonymous=True)
	# VisOdom = getVisualOdom()
	# rospy.spin()
	

	