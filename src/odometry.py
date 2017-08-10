#!/usr/bin/env python

# Yun Chang 2017 
  
import roslib
import rospy
import numpy as np 
from sensor_msgs.msg import Imu 
from nav_msgs.msg import Odometry 
from geometry_msgs.msg import Point, Quaternion, Vector3
import tf 

class arl_odomPublisher: 
	def __init__(self, topic='/odom', frame='/odom', child_frame='/base_link'):
		rospy.init_node('odom_generation', anonymous=True)
		self.pub = rospy.Publisher(topic, Odometry, queue_size=10)
		self.frame_id = frame
		self.child_frame_id = child_frame
		self.vodom_data = None 
		self.odom_data = None

	def CallVisOdom(self, data):
		try: # so code will keep running even if v_odom fails temporarily 
			vPos = data.pose.pose.position
			vOri = data.pose.pose.orientation
			covarPos = data.pose.covariance
			Twist = data.twist
			if covarPos[0] != 9999.0:
				self.vodom_data = [vPos, vOri, covarPos, Twist]
		except:
			print('no Viz Odom')

	def getVisualOdom(self, VisOdomTopic="viz_odom"):
		rospy.Subscriber(VisOdomTopic, Odometry, self.CallVisOdom)

	def Call2dOdom(self, data):
		oPos = data.pose.pose.position
		oOri = data.pose.pose.orientation
		self.odom_data = [oPos, oOri]

	def getOdom(self, OdomTopic="/camera/odom"):
		rospy.Subscriber(OdomTopic, Odometry, self.Call2dOdom)

	def changeAxis(self):
		if self.odom_data == None:
			raise ValueError
		# for the odom, the x points right and y points in front
		# so have it convert to the convention of rtabmap 
		# so newx = y, newy = -x, and maybe yaw? unknown 
		newx = self.odom_data[0].y
		newy = -self.odom_data[0].x
		return newx, newy

	def run(self):
		self.getOdom()
		self.getVisualOdom()
		while self.odom_data == None or self.vodom_data == None: 
			print('...')
		print('Odom Ready') 	
		r = rospy.Rate(30)
		while not rospy.is_shutdown():
			odomx, odomy = self.changeAxis()
			odom_quat = [self.odom_data[1].x, self.odom_data[1].y, self.odom_data[1].z, self.odom_data[1].w]
			vodom_quat = [self.vodom_data[1].x, self.vodom_data[1].y, self.vodom_data[1].z, self.vodom_data[1].w]
			odom_rpy = tf.transformations.euler_from_quaternion(odom_quat)
			vodom_rpy = tf.transformations.euler_from_quaternion(vodom_quat)
			odom_rpy = [odom_rpy[0], odom_rpy[1], odom_rpy[2]-(np.pi/2)]
			newx = odomx
			newy = odomy
			newYaw = odom_rpy[2]
			if self.vodom_data[0].x != 0.0:
				newx = (odomx + self.odom_data[0].x)/2. # average the x,y values for the vis_odom and odom 
			if self.vodom_data[0].y != 0.0:
				newy = (odomy + self.odom_data[0].y)/2.
			if vodom_rpy[2] != 0.0:
				newYaw = (odom_rpy[2] + vodom_rpy[2])/2. # also average yaw 
			quat = tf.transformations.quaternion_from_euler(vodom_rpy[0], vodom_rpy[1], newYaw)
			
			# publish odom topic 
			msg = Odometry()
			msg.header.stamp = rospy.Time.now()
			msg.header.frame_id = self.frame_id # i.e. '/odom'
			msg.child_frame_id = self.child_frame_id # i.e. '/base_link'

			msg.pose.pose.position = Point(newx, newy, self.vodom_data[0].z)
			msg.pose.pose.orientation = Quaternion(quat[0], quat[1], quat[2], quat[3])
			msg.pose.covariance = self.vodom_data[2]
			msg.twist = self.vodom_data[3]
			self.pub.publish(msg)
			self.getOdom()
			self.getVisualOdom()
			r.sleep()

if __name__ == '__main__':
	ijd = arl_odomPublisher()
	ijd.run()
