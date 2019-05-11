# coding=utf-8

from pblt import Interface,MOVABLE_ARM_JOINTS
import time
import numpy as np

from geometry import Point,Pose,Euler
from utils import Csv

def jaco():
    I=Interface()
    I.connect()

    #------------------------Jaco---------------------------#
    ROBOT_URDF_PATH ='../models/jaco_pkg/jaco_description/urdf/test_jaco.urdf'
    BLOCK_URDF = "../ss-pybullet-master/models/drake/objects/block_for_pick_and_place_mid_size.urdf"
    END_EFFECTOR_LINK = 'jaco_eef_link'
    TABLE = '../models/table/table.urdf'
    CUBE_URDF = '../models/cube/%s_cube.urdf'
    TRAY_URDF = '../models/tray/tray.urdf'
    BASKET_URDF = '../models/basket/basket.urdf'

    robot= I.load_model(ROBOT_URDF_PATH)

    redBlock1 = I.load_model(CUBE_URDF%'red',Pose(Point(0.22,-0.2,0.265)),fixed_base=False)
    basket =  I.load_model(BASKET_URDF, Pose(Point(0.35, 0.05, 0.265)), fixed_base = False)

    table = I.load_model(TABLE,Pose(Point(0.35,0.0,0.05),Euler(0.0,0.0,1.57)),fixed_base=False,scaling=0.3)

    #------------------------Jaco---------------------------#


    # velocities = Csv('velocities_UR5.csv',['joints_1','joints_2','joints_3','joints_4','joints_5','joints_6','joints_11','joints_13','joints_15','joints_16','joints_17','joints_18'],mode = 'r').read()
    # torques= Csv('torques_UR5.csv',['joints_1','joints_2','joints_3','joints_4','joints_5','joints_6','joints_11','joints_13','joints_15','joints_16','joints_17','joints_18'],mode = 'r').read()
    velocities = Csv('JACO_velocities.csv',['joints_3','joints_5','joints_7','joints_9','joints_11','joints_13','joints_16','joints_19','joints_22'],mode = 'r').read()
    torques = Csv('JACO_torques.csv',['joints_3','joints_5','joints_7','joints_9','joints_11','joints_13','joints_16','joints_19','joints_22'],mode = 'r').read()
    action = Csv('JACO_action.csv',['action'],mode = 'r').read()

    for joint,values in velocities.items():
        velocities[joint] = map(float,values)
    velocities = np.vstack(velocities.values())



    for joint,values in torques.items():
        torques[joint] = map(float,values)

    torques = np.vstack(torques.values())

    I.replay(robot,[velocities.T,torques.T,action.values()[0]])

    time.sleep(2.0)
    I.disconnect()

def ur5():
        I = Interface()
        I.connect()
        MOVABLE_ARM_JOINTS = [1, 2, 3, 4, 5, 6]
        ROBOT_URDF_PATH = '../models/ur5_description/urdf/ur5.urdf'
        END_EFFECTOR_LINK = 'end_effector_link'
        TABLE = '../models/table/table.urdf'
        CUBE_URDF = '../models/cube/%s_cube.urdf'

        # ------------------------UR5--------------------------#
        x_offset = 0.33
        y_offset = 0.15

        robot = I.load_model(ROBOT_URDF_PATH)

        redBlock1 = I.load_model(CUBE_URDF % 'red', Pose(Point(0.3 + x_offset, -0.1 + y_offset, 0.265)),
                                 fixed_base = False)
        redBlock2 = I.load_model(CUBE_URDF % 'red', Pose(Point(0.35 + x_offset, 0.05 + y_offset, 0.265)),
                                 fixed_base = False)
        table = I.load_model(TABLE, Pose(Point(0.35 + x_offset, 0.0 + y_offset, 0.05), Euler(0.0, 0.0, 1.57)),
                             fixed_base = False, scaling = 0.3)




        # ------------------------Jaco---------------------------#

        velocities = Csv('UR5_velocities.csv',
                         ['joints_1', 'joints_2', 'joints_3', 'joints_4', 'joints_5', 'joints_6', 'joints_11',
                          'joints_13', 'joints_15', 'joints_16', 'joints_17', 'joints_18'], mode = 'r').read()
        torques = Csv('UR5_torques.csv',
                      ['joints_1', 'joints_2', 'joints_3', 'joints_4', 'joints_5', 'joints_6', 'joints_11', 'joints_13',
                       'joints_15', 'joints_16', 'joints_17', 'joints_18'], mode = 'r').read()
        action = Csv('UR5_action.csv', ['action'], mode = 'r').read()

        for joint, values in velocities.items():
            velocities[joint] = map(float, values)
        velocities = np.vstack(velocities.values())

        for joint, values in torques.items():
            torques[joint] = map(float, values)

        torques = np.vstack(torques.values())

        I.replay(robot, [velocities.T, torques.T, action.values()[0]])
        #
        time.sleep(5.0)

        I.disconnect()


jaco()