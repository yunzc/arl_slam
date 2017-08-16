#!/usr/bin/env python
import rospy 
import roslib 
from geometry_msgs.msg import PoseWithCovarianceStamped as PCS 
from geometry_msgs.msg import Vector3, Point
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import Range 
import tf 

# short script to convert from msg type PoseWithCovarianceStamped to Odom message 
# will also incorporate range sensor data here 

integrate_range_sensor = False
odom_pos = None 
odom_ori = None 
rs_height = None
odom_msg = None
odom_cov = None

#define callback function 
def callBackPCS(data): # call back for odom data 
	odom_data = data # extract data to use 

	global odom_pos
	global odom_ori
	global odom_cov

	odom_pos = odom_data.pose.pose.position 
	odom_ori = odom_data.pose.pose.orientation 
	odom_cov = odom_data.pose.covariance

def callBackRS(data): # call back for Range Sensor 
	global rs_height
	rs_height = data.range 

def sensor_fusion(pos_odom, height_rs, rs_minRange=0.2, rs_maxRange=14.0, rs_weight=0.1, odom_weight=0.9):
	print('height_rs')
	print(height_rs)
	print(pos_odom)
	# if height_rs < (rs_minRange + 0.1) or height_rs > (rs_maxRange - 0.1):
	# 	return [pos_odom.x, pos_odom.y, pos_odom.z]
	# else:
	# 	new_z = rs_weight*rs_height + odom_weight*pos_odom.z 
	# 	return [pos_odom.x, pos_odom.y, new_z]
	return [pos_odom.x, pos_odom.y, pos_odom.z]

def create_odom_msg(pos, ori, cov):
	msg = Odometry()
	msg.header.stamp = rospy.Time.now()
	msg.header.frame_id = '/odom' # i.e. '/odom'
	msg.child_frame_id = '/camera_link' # i.e. '/base_link'
	msg.pose.pose.position = pos
	msg.pose.pose.orientation = ori 
	msg.pose.covariance = cov

	euler = tf.transformations.euler_from_quaternion([ori.x, ori.y, ori.z, ori.w])
	msg.twist.twist.linear = Vector3(pos.x, pos.y, pos.z)
	msg.twist.twist.angular = Vector3(euler[0], euler[1], euler[2])
	msg.twist.covariance = cov

	return msg

# init note, subscribe to data and publish 
rospy.init_node('pcs2odom', anonymous=True)
rospy.Subscriber('/camera/robot_pose_ekf/odom_combined', PCS, callBackPCS)
if integrate_range_sensor:
	rospy.Subscriber('/terangerone', Range, callBackRS)
pub = rospy.Publisher('/odom', Odometry, queue_size=10)
tf_br = tf.TransformBroadcaster()

# rate 
r = rospy.Rate(30)
pos = None
new_msg = None 
while not rospy.is_shutdown():
	if odom_pos != None: 
		if rs_height != None:
			pos = sensor_fusion(odom_pos, rs_height)
			pt_pos = Point(pos[0], pos[1], pos[2])
			new_msg = create_odom_msg(pt_pos, odom_ori, odom_cov)
		else:
			pos = [odom_pos.x, odom_pos.y, odom_pos.z]
			new_msg = create_odom_msg(odom_pos, odom_ori, odom_cov)
		pub.publish(new_msg)
		q = [odom_ori.x, odom_ori.y, odom_ori.z, odom_ori.w]
		tf_br.sendTransform(pos, q, rospy.Time.now(), '/camera_link', '/odom')
	r.sleep()
