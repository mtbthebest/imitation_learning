# coding=utf-8

from pblt import Interface
import numpy as np
from utils import Pd, get_folders
import time

from collections import namedtuple
from geometry import Point, Pose, Euler
import os
import socket

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.connect(("8.8.8.8", 80))
if s.getsockname()[0] == '192.168.100.177':
    DATA_PATH = '/home/mtb/sim_ws/pybullet_ws/data/'

elif s.getsockname()[0] == '192.168.1.20':
    DATA_PATH = '/media/mtb/76f4874c-25bd-4dd1-b7a5-dc5732ba8511/mtb/THROW/'
s.close()

ROBOT_URDF_PATH = '../models/jaco_pkg/jaco_description/urdf/test_jaco.urdf'
END_EFFECTOR_LINK = 'jaco_eef_link'

TABLE = '../models/cube_table/cube_table.urdf'

CUBE_URDF = '../models/cube/red_cube.urdf'

TABLE_HEIGHT = 0.126

limits = {'x_min': 0.2, 'x_max': 0.35,
          'y_min': -0.3, 'y_max': 0.3}
block_thresh = 0.125

TRIALS = 5

BLOCKS = ["%(color)sBlock%(indice)d" % ({'color': name, 'indice': indice}) for name in ['red', 'green', 'blue'] for
          indice in [1, 2]]
BLOCK_SIZE = 0.05
Block = namedtuple('Block', ['pose', 'handle', 'name'])

BASKET_URDF = '../models/basket/basket.urdf'



def inside_basket(I, *args):
    """ Check if the block has fallen"""
    block_handle = args[0]
    basket_handle = args[1]
    #
    block1_pose = I.get_position(block_handle)[0]
    basket_pose = I.get_position(basket_handle)[0]

    if block1_pose[0] - 0.1 <= basket_pose[0] <= block1_pose[0] + 0.1 and \
            block1_pose[1] - 0.1 <= basket_pose[1] <= block1_pose[1] + 0.1:
        return True
    else:
        return False


def main():
    folders = get_folders(path=DATA_PATH)
    p_mode_velocity_file = 'p_mode_velocities.csv'
    scene_objects = 'scene_objects.npy'
    success = 0
    for i in range(507, 999):
        folder = DATA_PATH +'Trial_%d/'%i
        with Interface() as I:
            # LOAD ROBOT
            robot = I.load_model(ROBOT_URDF_PATH)

            # LOAD TABLE
            table = I.load_model(TABLE, Pose(Point(0.4, 0.0, 0.05), Euler(1.57, 0.0, 0.0)),
                                 fixed_base=True, scaling=1.0)

            # TODO set camera
            I.set_camera_pose(0.6, 155, -50, [0.2, 0.0, 0.5])

            scene = np.load(os.path.join(folder, scene_objects))[2:, 0]

            block = I.load_model(CUBE_URDF, Pose(Point(*scene[0])), fixed_base=False)
            basket = I.load_model(BASKET_URDF, Pose(Point(*scene[1])), fixed_base=False)

            I.save_body_conf()


            if os.path.isfile(os.path.join(folder, p_mode_velocity_file)):
                velocities = Pd(os.path.join(folder , p_mode_velocity_file)).read().values
                I.replay_velocities(robot,velocities)

                if inside_basket(I,block,basket):
                    I.restore_body_conf()
                    time.sleep(1.0)

                    I.replay_velocities(robot,velocities,filename=os.path.join(folder,'img'))
                    success +=1

        if success ==270:
            break

        print('Trials : ', folder, ' , Success: ', success)


if __name__ == '__main__':
    main()
