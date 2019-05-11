#!/usr/bin/env python

import rospy
from bot_gazebo.srv import ImageState, ImageStateResponse, ImageStateRequest
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import cv2
import rosbag

def get_states(request):
	
	# cv2.namedWindow("window", 1)
	# bridge = CvBridge()
	try:
		msg = rospy.wait_for_message('/head_camera/rgb/image_color', Image, timeout = 10.0)
		with rosbag.Bag('state.bag', 'w') as outbag:
			outbag.write('/head_camera/rgb/image_color', msg)
			rospy.sleep(1.0)
		# try:
		# 	image = bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
		#
		# except CvBridgeError, e:
		# 	image = None
		#
		# if image is not None:
		# 	image = cv2.resize(image, (84, 84))
		# 	cv2.imshow('window',image)
		#
		# 	cv2.imwrite('state.png', image)
		return (ImageStateResponse('success'))

	except:
		return (ImageStateResponse('error'))


if __name__ == '__main__':
	rospy.init_node('vision_server')
	model_state_service = rospy.Service('state', ImageState, get_states)
	i = 0
	print('Starting model state service')
	rospy.spin()
