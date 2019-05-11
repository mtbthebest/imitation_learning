# coding=utf-8

from pblt import Interface, MOVABLE_ARM_JOINTS
import os
import numpy as np
import time
import random
from math import sqrt
from collections import namedtuple
from geometry import Point, Pose, Euler
from utils import mkdir, folder_exists, pathname
from random import shuffle
from collections import OrderedDict
import shutil
import json

import socket

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
IP = s.getsockname()[0]
s.close()

import sys

ROBOT_URDF_PATH = '../models/jaco_pkg/jaco_description/urdf/test_jaco.urdf'
END_EFFECTOR_LINK = 'jaco_eef_link'

TABLE = '../models/cube_table/cube_table.urdf'

CUBE_URDF = '../models/cube/%s_cube.urdf'

TABLE_HEIGHT = 0.126

limits = {'x_min': 0.2, 'x_max': 0.35,
          'y_min': -0.3, 'y_max': 0.3}
block_thresh = 0.125


BLOCKS = ["%(color)sBlock%(indice)d" % ({'color': name, 'indice': indice}) for name in ['red'] for
          indice in [1, 2]]
BLOCK_SIZE = 0.05
Block = namedtuple('Block', ['pose', 'handle', 'name'])

DATA_PATH = '/home/mtb/sim_ws/pybullet_ws/data/'


def load_blocks(I):
    """
    Load the blocks
    :param I: Interface
    :return: list of blocks pose and handle parameters
    """
    res = []
    for block in BLOCKS:
        j = 0
        while True:
            r_x = limits['x_min'] + (limits['x_max'] - limits['x_min']) * random.random()
            r_y = limits['y_min'] + (limits['y_max'] - limits['y_min']) * random.random()
            collision = False

            for b in res:
                if sqrt((b.pose[0] - r_x) ** 2 + (b.pose[1] - r_y) ** 2) <= block_thresh:
                    collision = True
                    break
            if not collision:
                pose = [r_x, r_y, TABLE_HEIGHT]
                handle = I.load_model(CUBE_URDF % block[0:block.find('Block')], Pose(Point(*pose)), fixed_base=False)
                res.append(Block(pose=pose, handle=handle, name=block[:block.find('Block')]))
                break
            if j == 1000:
                break
            j += 1

    return res


def plan(I, robot, gripper_link, b, neutralPose, initialize=False, x_offset=0.0, y_offset=0.0):
    initial_pose = [neutralPose] if initialize else []

    target_pose = initial_pose + \
                  [Pose(Point(b[0].pose[0], b[0].pose[1], TABLE_HEIGHT + 0.2), Euler(0.0, 3.14, 1.57)),
                   Pose(Point(b[0].pose[0], b[0].pose[1], TABLE_HEIGHT + 0.02), Euler(0.0, 3.14, 1.57)),
                   Pose(Point(b[0].pose[0], b[0].pose[1], TABLE_HEIGHT + 0.2), Euler(0.0, 3.14, 1.57)),

                   Pose(Point(b[1].pose[0] + x_offset, b[1].pose[1] + y_offset, TABLE_HEIGHT + 0.3),
                        Euler(0.0, 3.14, 1.57)),
                   Pose(Point(b[1].pose[0] + x_offset, b[1].pose[1] + y_offset, TABLE_HEIGHT + 0.07),
                        Euler(0.0, 3.14, 1.57)),
                   Pose(Point(b[1].pose[0], b[1].pose[1], TABLE_HEIGHT + 0.3), Euler(0.0, 3.14, 1.57)),

                   ]
    stack_path = []
    for poses in target_pose:
        end_conf = I.calculate_inverse_kinematics(robot, gripper_link, poses)

        if end_conf is not None:
            path = I.plan_motion(robot, end_conf, [2, 3, 4, 5])
            if path is None:
                return None
            else:
                stack_path.append(path)
        else:
            return None
        I.set_joint_positions(robot, MOVABLE_ARM_JOINTS, end_conf)

    return stack_path


def on_top(I, *args):
    """ Check if the block has fallen"""
    block1_handle = args[0].handle
    block2_handle = args[1].handle

    block1_pose = I.get_position(block1_handle)
    block2_pose = I.get_position(block2_handle)

    if block1_pose[0][2] >= block2_pose[0][2] + 0.04:
        return True
    else:
        return False


def main():
    with open(pathname(os.getcwd(), '..', '..', 'machines.json'), 'r') as f:
        js = json.load(f)
    DATA_PATH = js[IP]['DATA_PATH']
    trial = js[IP]['trial_start']
    while True:
        if trial == js[IP]['trial_stop']:
            break
        start = time.time()
        while time.time() - start <= 120.0:
            SAVE_PATH = DATA_PATH + 'Trial_%d/' % trial
            mkdir(SAVE_PATH)
            p_mode_velocity_file = SAVE_PATH + 'p_mode_velocities.csv'
            scene_object_file = SAVE_PATH + 'scene'

            with Interface(mode='direct', save=True,
                           filename=p_mode_velocity_file) as I:
                # LOAD ROBOT
                robot = I.load_model(ROBOT_URDF_PATH)

                # LOAD TABLE
                table = I.load_model(TABLE, Pose(Point(0.4, 0.0, 0.05), Euler(1.57, 0.0, 0.0)),
                                     fixed_base=True, scaling=1.0)

                # TODO set camera
                I.set_camera_pose(1.6, 60, -25, [0.2, 0.0, 0.7])

                # LOAD_BLOCKS
                blocks = load_blocks(I)
                if len(blocks) != len(BLOCKS):
                    continue
                time.sleep(1.0)

                # Gripper ID
                gripper_link = I.get_link_from_name(robot, END_EFFECTOR_LINK)

                # Neutral Pose
                neutralPose = Pose(Point(0.35, 0.0, 0.5), Euler(0.0, 3.14, 1.57))

                I.save_body_conf(save=True, filename=scene_object_file)
                I.set_initial_state(robot)

                offset = 0.005
                x_offset = [0.0, offset, -offset, -offset]
                y_offset = [0.0, offset, -offset, 0.0]

                shuffle(blocks)

                color = ['red']

                for k in range(1):
                    # Set initial state
                    I.restore_body_conf()
                    success = False

                    # Plan motion
                    paths = []
                    velocities = OrderedDict()

                    for j in range(1):
                        initialize = True if j == 0 else False
                        path = plan(I, robot, gripper_link, blocks, neutralPose,
                                    initialize=initialize, x_offset=x_offset[k], y_offset=y_offset[k])
                        if path is not None:
                            paths.append(path)
                        else:
                            paths = []
                            break

                    if len(paths) > 0:
                        I.reset_simulation(robot)
                        # time.sleep(1.0)

                        I.command(robot, paths[0][0])
                        I.command(robot, paths[0][1])
                        I.command(robot, paths[0][2])
                        I.gripper(robot, 'close')
                        I.command(robot, paths[0][3])
                        I.command(robot, paths[0][4])
                        I.command(robot, paths[0][5])

                        I.gripper(robot, 'open')
                        I.command(robot, paths[0][6])

                        if on_top(I, *blocks[0:2]):
                            I.keep_pose(body=robot)
                            I.vel.write_csv()

                            velocities = I.vel.array
                            I.restore_body_conf()
                            I.reset_simulation(robot)

                            I.replay_velocities(robot, velocities)

                            if on_top(I, *blocks[0:2]):
                                I.restore_body_conf()
                                I.reset_simulation(robot)
                                time.sleep(1.0)
                                I.replay_velocities(robot,
                                                    velocities,
                                                    path=os.path.abspath(SAVE_PATH))

                                if on_top(I, *blocks[0:2]):
                                    success = True
                                    np.save(os.path.join(SAVE_PATH, 'velocities'), np.array(I.target_vel))
                                    np.save(os.path.join(SAVE_PATH, 'joints_configurations'), np.array(I.joint_val))
                                    trial += 1
                                    break

                else:
                    shutil.rmtree(SAVE_PATH)
            if success:
                break
            else:
                if folder_exists(SAVE_PATH):
                    shutil.rmtree(SAVE_PATH)




if __name__ == '__main__':
    main()
