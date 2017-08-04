#!/usr/bin/env python  
import roslib
import rospy
import numpy as np 
from sensor_msgs.msg import Imu 
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Point, Quaternion, Vector3
import tf 

class arl_odomPublisher: 
	def __init__(self, topic='/odom', frame='/odom', child_frame='/camera_link'):
		rospy.init_node('kf', anonymous=True)
		self.br = tf.TransformBroadcaster()
		self.pub = rospy.Publisher(topic, Odometry, queue_size=10)

		self.frame_id = frame
		self.child_frame_id = child_frame
		self.vodom_data = None 
		self.imu_data = None

	def CallVisOdom(self, data):
		# return [linear, angular, covar] 
		lin = data.twist.twist.linear
		ang = data.twist.twist.angular
		covar = np.matrix(data.twist.covariance)
		covar = covar.reshape([6,6])
		lincov = covar[0:3,0:3]
		angcov = covar[3:, 3:]
		global vodom_data
		linpos = [lin.x, lin.y, lin.z]
		angpos = [ang.x, ang.y, ang.z]
		self.vodom_data = [linpos, angpos, lincov, angcov]

	def getVisualOdom(self, VisOdomTopic="viz_odom"):
		rospy.Subscriber(VisOdomTopic, Odometry, self.CallVisOdom)

	def CallImu(self, data):
		angVel = data.angular_velocity 
		angCov = np.matrix(data.angular_velocity_covariance)
		angCov = angCov.reshape([3,3])
		linAcc = data.linear_acceleration
		linCov = np.matrix(data.linear_acceleration_covariance)
		linCov = linCov.reshape([3,3])
		aV = np.array([[angVel.x],[angVel.y],[angVel.z]])
		lA = np.array([[linAcc.x],[linAcc.y],[linAcc.z]])
		global imu_data
		self.imu_data = [aV, angCov, lA, linCov]

	def getImuData(self, ImuTopic="/camera/imu/data_raw"):
		rospy.Subscriber(ImuTopic, Imu, self.CallImu)

	@staticmethod
	def computeEstimateTranslation(U, Q, Z, P, R, xt_prev, d_t):
		# the states i want will be x, xdot, y, ..., zdot
		# and yaw pitch roll with their velocities 
		# but i shall turn the two into independent computations 
		# input previous translational state vector and rotational 
		# state vector 
		# Covariance from imu only give lives
		# define identity matrix 
		I = np.matrix(np.identity(6))
		F_t = np.matrix([[1, d_t, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, \
			1, d_t, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, d_t], [0, \
			0, 0, 0, 0, 1]]) # transform matrix from previous state 
		B_t = np.matrix([[.5*d_t**2, 0, 0], [d_t, 0, 0], [0, .5*d_t**2, 0], \
			[0, d_t, 0], [0, 0, .5*d_t**2], [0, 0, d_t]]) # mapping from input (in this case accel)
		xt_ = F_t*xt_prev + B_t*U # first translational estimation 
		H = I # In our case the "measurement" is visual odom 
		# Q_ = B_t*Q*np.matrix.transpose(B_t)
		P_ = F_t*P*np.matrix.transpose(F_t) + Q
		# R_ = B_t*R*np.matrix.transpose(B_t)
		S = H*P_*np.matrix.transpose(H) + R
		K = P_*np.matrix.transpose(H)*np.linalg.inv(S) # Kalman gain 
		y = Z - H*xt_prev
		x_est = xt_ + K*y 
		P_est = (I - K*H)*P_
		return x_est, P_est 

	@staticmethod
	def computeEstimateRotation(U, Q, Z, P, R, xr_prev, d_t):
		# define identity matrix 
		I = np.matrix(np.identity(3))
		F_r = I
		B_r = d_t*I 
		xr_ = F_r*xr_prev + B_r*U # first rotational estimation 
		H = I 
		P_ = F_r*P*np.matrix.transpose(F_r) + Q 
		S = H*P_*np.matrix.transpose(H) + R 
		K = P_*np.matrix.transpose(H)*np.linalg.inv(S)
		y = Z - H*xr_prev
		x_est = xr_ + K*y 
		P_est = (I - K*H)*P_ 
		return x_est, P_est

	def run(self):
		xt = np.zeros([6,1])
		xr = np.zeros([3,1])
		Pt = np.matrix(np.identity(6))*100.
		Pr = np.matrix(np.identity(3))*100.
		r = rospy.Rate(40) #define rate 
		dt = 1./20
		self.getImuData()
		self.getVisualOdom()
		while self.imu_data == None or self.vodom_data == None: 
			print('...')
		print('Ready')
		Ut = self.imu_data[2] # get control: translational
		Ur = self.imu_data[0] # get control: rotational 
		Qr = self.imu_data[1] # get control covariance 
		vt = self.vodom_data[0] # get visual odom pose 
		vr = self.vodom_data[1] # get rotational pose from v-odom 	
		while not rospy.is_shutdown():
			self.getImuData()
			self.getVisualOdom()
			if self.imu_data != None:
				Ut = self.imu_data[2] # get control: translational
				Ur = self.imu_data[0] # get control: rotational 
				Qr = self.imu_data[1] # get control covariance 
			if self.vodom_data != None:
				vt = self.vodom_data[0] # get visual odom pose 
				vr = self.vodom_data[1] # get rotational pose from v-odom 
			# convert to array format 
			Zt = np.array([[vt[0]], [xt[1,0]], [vt[1]], [xt[3,0]], [vt[2]], [xt[5,0]]])
			Zr = np.array([[vr[0]], [vr[1]], [vr[2]]])
			Rt = np.matrix(np.identity(6)) #prepare covar matrix
			Rr = self.vodom_data[3] 
			# note 'augmenting' is only neccessary because we included linear velocity in our state 
			# so we have to 'augment' the covar matrix. but it's ok since we are unsure of velocity, just make related
			# entries have big values 
			# no need to touch rotational covar since it has the right dim 
			Qt = np.matrix(np.identity(6)) 
			for i in range(3):
				Rt[2*i, 2*i] = self.vodom_data[2][i,i]
				Qt[2*i, 2*i] = self.imu_data[3][i,i]
			xt, Pt = self.computeEstimateTranslation(Ut, Qt, Zt, Pt, Rt, xt, dt)
			xr, Pr = self.computeEstimateRotation(Ur, Qr, Zr, Pr, Rr, xr, dt)
			pos = (xt[0,0], xt[1,0], xt[2,0])
			quat = tf.transformations.quaternion_from_euler(xr[0,0], xr[1,0], xr[2,0])
			# tf
			self.br.sendTransform(pos, quat, rospy.Time.now(), self.child_frame_id, self.frame_id)
			# publish odom topic 
			msg = Odometry()
			msg.header.stamp = rospy.Time.now()
			msg.header.frame_id = self.frame_id # i.e. '/odom'
			msg.child_frame_id = self.child_frame_id # i.e. '/base_footprint'

			msg.pose.pose.position = Point(pos[0], pos[1], pos[2])
			msg.pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
			Cov = np.identity(6) # complete covariance matrix
			Cov[0:3,0:3] = Pt[::2,::2]
			Cov[3:,3:] = Pr
			msg.pose.covariance = tuple(np.ravel(Cov).tolist()) # turn into tuple 
			msg.twist.twist.linear = Vector3(pos[0], pos[1], pos[2])
			msg.twist.twist.angular = Vector3(xr[0,0],xr[1,0],xr[2,0])
			msg.twist.covariance = tuple(np.ravel(Cov).tolist()) # turn into tuple 
			self.pub.publish(msg)
			r.sleep()




if __name__ == '__main__':
	ijd = arl_odomPublisher()
	ijd.run()
	# rospy.init_node('kf', anonymous=True)
	# br = tf.TransformBroadcaster()
	# while not rospy.is_shutdown():
	# 	pos = (0,0,0)
	# 	quat = (0,0,0,1)
	# 	br.sendTransform(pos, quat, rospy.Time.now(), 'odom', 'camera_link')
