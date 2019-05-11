#!/usr/bin/env	python

import cv2

vidcap = cv2.VideoCapture('/home/mtb/recording_2018_09_07-22_35-42.avi')
success, image = vidcap.read()

count = 0
success = True
while success:
	cv2.imwrite("../jaco_demo/frame%d.jpg" % count, image)  # save frame as JPEG file
	success, image = vidcap.read()
	
	count += 1