#!/usr/bin/env python

# this node is to just convert datatype to make compatible the realsense px4 nuc system 
# mavros_msgs/Altitude -> geometry_msgs/PointStamped
# Odom -> geometry_msgs/PoseWithCovarianceStamped 

from mavros_msgs.msg import Altitude
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion, Vector3, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf 
import rospy 
import roslib 

msgAlt = None
msgPose = None 
rospy.init_node('converter', anonymous=True)
pubAlt = rospy.Publisher('/mavros/height', PointStamped, queue_size=10)
pubPose = rospy.Publisher('/camera/vodom', PoseWithCovarianceStamped, queue_size=10)

def callBackAltitude(data):
	# Altitude msg to PointStamped 
	h = data.local 
	msg = PointStamped()
	msg.header = data.header
	msg.point = Point(0,0,h)
	global msgAlt
	msgAlt = msg 
	

def callBackPose(data):
	# Odom msg to Pose with Covariance Stamped 
	pose = data.pose
	msg = PoseWithCovarianceStamped()
	msg.header = data.header
	msg.pose = pose
	global msgPose
	msgPose = msg 

rate = rospy.Rate(30)
rospy.Subscriber('/mavros/altitude', Altitude, callBackAltitude)
rospy.Subscriber('/camera/odom', Odometry, callBackPose)
while not rospy.is_shutdown():
	if msgAlt != None and msgPose != None:
		pubAlt.publish(msgAlt)
		pubPose.publish(msgPose)
	rate.sleep()