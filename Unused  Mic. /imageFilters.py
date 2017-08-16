#!/usr/bin/env python

# Yun Chang 2017
# filter the depth and color images from the realsense hope it will make 
# SLAM results better 

import rospy
import roslib 
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class ImageFilter:
    def __init__(self, filter_size=5):
        rospy.init_node("image_filter", anonymous=True)
        self.subRGB = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.callBackRGB)
        self.subDepth = rospy.Subscriber("/camera/depth_registered/sw_registered/image_rect_raw", Image, self.callBackDepth)
        self.pubRGB = rospy.Publisher("/rgb/image_filtered", Image, queue_size=10)
        self.pubDepth = rospy.Publisher("/depth/image_filtered", Image, queue_size=10)
        self.bridge = CvBridge()
        self.blur_size = filter_size
        self.RGBimage = None 
        self.Depthimage = None # eventually want this in cv2 image form 

    def callBackRGB(self, data):
        try:
            rgb_img = self.bridge.imgmsg_to_cv2(data,"passthrough")
        except CvBridgeError as e:
            print(e)

        rgb_img = np.nan_to_num(rgb_img)
        #median filter 
        rgb_img_filt = cv2.medianBlur(rgb_img, self.blur_size)

        try:
            msg = self.bridge.cv2_to_imgmsg(rgb_img_filt, "passthrough")
            self.pubRGB.publish(msg)
        except CvBridgeError as e:
            print(e)

    def callBackDepth(self, data):
        try:
            depth_img = self.bridge.imgmsg_to_cv2(data,"passthrough")
        except CvBridgeError as e:
            print(e)

        depth_img = np.nan_to_num(depth_img)
        # apply median filter
        depth_img_filt = cv2.medianBlur(depth_img, self.blur_size)

        try:
            msg = self.bridge.cv2_to_imgmsg(depth_img_filt, "passthrough")
            self.pubDepth.publish(msg)
        except CvBridgeError as e:
            print(e)


if __name__ == "__main__":
    sf = ImageFilter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
cv2.destroyAllWindows()