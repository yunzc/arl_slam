#!/usr/bin/env python
import rospy 
import roslib 
from geometry_msgs.msg import PoseWithCovarianceStamped as PCS 
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry 
import tf 

# short script to convert from msg type PoseWithCovarianceStamped to Odom message 
odom_msg = None

#define callback function 
def callBack(data):
	odom_data = data # extract data to use 

	pos = odom_data.pose.pose.position 
	ori = odom_data.pose.pose.orientation 

	msg = Odometry()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = '/odom' # i.e. '/odom'
	msg.child_frame_id = '/camera_link' # i.e. '/base_link'
	msg.pose = odom_data.pose 

	euler = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
	msg.twist.twist.linear = Vector3(pos.x, pos.y, pos.z)
	msg.twist.twist.angular = Vector3(euler[0], euler[1], euler[2])
	msg.twist.covariance = odom_data.pose.covariance 
	# store message to publish 
	global odom_msg
	odom_msg = msg

	return True 

# init note, subscribe to data and publish 
rospy.init_node('pcs2odom', anonymous=True)
rospy.Subscriber('/camera/robot_pose_ekf/odom_combined', PCS, callBack)
pub = rospy.Publisher('/odom', Odometry, queue_size=10)

# rate 
r = rospy.Rate(30)
while not rospy.is_shutdown():
	if odom_msg != None: 
		pub.publish(odom_msg)
	r.sleep()
