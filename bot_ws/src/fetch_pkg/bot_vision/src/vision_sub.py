#!/usr/bin/env	python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import os

TOPIC_NAME = 'head_camera/rgb/image_color' #'/xtion/rgb/image_raw'#'/cameras/head_camera/image'  ,
class Vision:
	def __init__(self):
		rospy.init_node('vision')
		self.bridge = CvBridge()

		self.image_sub = rospy.Subscriber(TOPIC_NAME,Image, self.image_callback)
		rospy.spin()

	def image_callback(self, msg):
		self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
		cv2.imshow("window", self.image)
		cv2.waitKey(1)




if __name__ == '__main__':
	Vision()
