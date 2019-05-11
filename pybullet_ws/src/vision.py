# coding=utf-8


from pblt import Interface, MOVABLE_ARM_JOINTS
import pybullet
import os
import time
import random
from math import sqrt
from collections import namedtuple
from geometry import Point, Pose, Euler
from utils import mkdir

import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
if s.getsockname()[0] == '192.168.100.177':
    DATA_PATH = '/home/mtb/sim_ws/pybullet_ws/data/'
elif s.getsockname()[0] == '192.168.1.12':
    DATA_PATH = '/media/mtb/Data Disk/JACO/DEMO/STACK/'
s.close()

ROBOT_URDF_PATH = '../models/jaco_pkg/jaco_description/urdf/test_jaco.urdf'
END_EFFECTOR_LINK = 'jaco_eef_link'

TABLE = '../models/cube_table/cube_table.urdf'

CUBE_URDF = '../models/cube/%s_cube.urdf'

TABLE_HEIGHT = 0.126


limits = {'x_min':0.2,'x_max':0.35,
          'y_min':-0.3,'y_max':0.3}
block_thresh = 0.125

TRIALS = 10000

BLOCKS =["%(color)sBlock%(indice)d" % ({'color': name ,'indice':indice})for name in ['red','green','blue'] for indice in [1,2]]
BLOCK_SIZE = 0.05
Block = namedtuple('Block',['pose','handle'])

I = Interface()
I.connect()
# LOAD ROBOT
robot = I.load_model(ROBOT_URDF_PATH)

# LOAD TABLE
table = I.load_model(TABLE, Pose(Point(0.4, 0.0, 0.05), Euler(1.57, 0.0, 0.0)),
                     fixed_base = True, scaling = 1.0)

# TODO set camera
camTargetPos = [0, 0, 0]
cameraUp = [0, 0, 1]
cameraPos = [1, 1, 1]

pitch = -50
roll = 10
yaw =70
upAxisIndex = 2
camDistance = 1.5
pixelWidth = 256
pixelHeight = 256
nearPlane = 0.01
farPlane = 100
fov = 60

for i in range(0,100):
    start = time.time()
    viewMatrix = pybullet.computeViewMatrixFromYawPitchRoll(camTargetPos, camDistance, yaw, pitch, roll,
                                                            upAxisIndex)
    aspect = pixelWidth / pixelHeight;
    projectionMatrix = pybullet.computeProjectionMatrixFOV(fov, aspect, nearPlane, farPlane);
    img_arr = pybullet.getCameraImage(pixelWidth, pixelHeight, viewMatrix,
                                      projectionMatrix, shadow = 1, lightDirection = [1, 1, 1],
                                      renderer = pybullet.ER_BULLET_HARDWARE_OPENGL)

    print time.time() - start

I.disconnect()