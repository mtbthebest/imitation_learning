#!/usr/bin/env	python
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import numpy as np
from jaco_vrep_msgs.msg import Torques
from utils import  Csv

class Vision:
    def __init__(self):
        rospy.init_node('vision')
        self.bridge = CvBridge()
        self.i = 0
        self.image_sub = rospy.Subscriber('/image',Image, self.image_callback)
        rospy.spin()

    def image_callback(self, msg):
        try:
            img_des = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')
            img_np = np.array(img_des, np.uint8).reshape([msg.height, msg.width, -1])
            np.save('/media/mtb/Data Disk/JACO/DEMO/STACKING/DEMO_0/img/img_%d'%self.i,img_np)
            self.i +=1
        except CvBridgeError as e:
            pass
class Torque:
    def __init__(self):
        rospy.init_node('torque')
        self.headers = ['joints%d'%i for i in range(6)] + ['timestep_sec','timestep_nsec']
        self.filename='/media/mtb/Data Disk/JACO/DEMO/STACKING/DEMO_0/torques/torques_ur5.csv'
        self.writer= Csv(self.filename,self.headers,'a')

        rospy.Subscriber('/torques',Torques, self.torque_callback)
        rospy.spin()

    def torque_callback(self, msg):

            timestep_sec  = float(msg.header.stamp.secs)
            timestep_nsec  = float(msg.header.stamp.nsecs)

            torques = msg.data
            data =list(torques)+[timestep_sec,timestep_nsec]
         
            self.writer.write([data])



if __name__ == '__main__':
    Torque()
