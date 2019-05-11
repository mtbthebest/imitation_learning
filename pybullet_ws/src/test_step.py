# coding=utf-8
"""Visuomotor network"""

import os
import numpy as np
from utils import pathname, mkdir, Img
from utils import Pd, get_files
from bullet import Interface, Point, Pose, Euler
import scipy.misc


WIDTH = 256
HEIGHT = 256
CHANNELS = 3

# Testing parameters
ROBOT_URDF_PATH = '/home/mtb/sim_ws/pybullet_ws/models/jaco_pkg/jaco_description/urdf/test_jaco.urdf'
END_EFFECTOR_LINK = 'jaco_eef_link'
TABLE = '/home/mtb/sim_ws/pybullet_ws/models/cube_table/cube_table.urdf'
CUBE_URDF = '/home/mtb/sim_ws/pybullet_ws/models/cube/red_cube.urdf'
BASKET_URDF = '/home/mtb/sim_ws/pybullet_ws/models/basket/basket.urdf'
DATA_PATH = '/home/mtb/'


folder = DATA_PATH + 'Trial_1110'
p_mode_velocity_file = 'p_mode_velocities.csv'
scene_objects = 'scene_objects.npy'
with Interface(mode = 'gui') as I:
    robot = I.load_model(ROBOT_URDF_PATH)
    table = I.load_model(TABLE, Pose(Point(0.4, 0.0, 0.05), Euler(1.57, 0.0, 0.0)),
                         fixed_base = True, scaling = 1.0)
    I.set_camera_pose(0.6, 155, -50, [0.2, 0.0, 0.5])
    scene = np.load(os.path.join(folder, scene_objects))[2:, 0]
    block = I.load_model(CUBE_URDF, Pose(Point(*scene[0])), fixed_base = False)
    basket = I.load_model(BASKET_URDF, Pose(Point(*scene[1])), fixed_base = False)


    velocity =np.load(pathname(folder,'velocities.npy'))

    STEPS=velocity.shape[0]
    # img = np.concatenate([[np.load(pathname(folder,'IMG', 'img_%d.npy'%i))[:,:,:3] for i in range(STEPS)]])
    # joints = I.movable_a_joints + I.movable_g_joints
    # vels = []
    for i in range(STEPS):
        print i
    #     joints_configurations = [[state.jointPosition for state in I.get_joint_state(robot, joints)]]
    #     rgb = self.sess.run(tf.image.per_image_standardization(img[i]))
    #     vel = self.sess.run(self.vel,
    #         {
    #         self.rgb:rgb.reshape(-1,WIDTH,HEIGHT,CHANNELS),
    #         self.conf:joints_configurations
    #     }
    #     )
    #     vels.append(vel[0])

    #
        I.step(body = robot, values = velocity[i].reshape(1,9))
    #
        img_array = I.get_camera_image(WIDTH,HEIGHT)[:,:,:3]
        scipy.misc.imsave('/home/mtb/Correct_eval/img_%d.png'%(i), img_array)
    #
    # np.save('/data/barry/IMITATION/dataset/THROW/EVALUATION/vels', np.array(vels))






