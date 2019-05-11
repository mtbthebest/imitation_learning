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

END_EFFECTOR_LINK = 'jaco_eef_link'

TABLE = '../models/cube_table/cube_table.urdf'
ROBOT_URDF_PATH = '../models/jaco_pkg/jaco_description/urdf/test_jaco.urdf'

CUBE_URDF = '/home/mtb/sim_ws/pybullet_ws/models/cube/red_cube.urdf'
SPHERE_URDF = '/home/mtb/sim_ws/pybullet_ws/models/sphere/sphere2.urdf'
GSPHERE_URDF = '/home/mtb/sim_ws/pybullet_ws/models/sphere_green/sphere2.urdf'
CYLINDER_URDF = '/home/mtb/sim_ws/pybullet_ws/models/cylinder/cylinder.urdf'
GCYLINDER_URDF = '/home/mtb/sim_ws/pybullet_ws/models/cylinder_green/cylinder.urdf'
GREEN_CUBE_URDF = '/home/mtb/sim_ws/pybullet_ws/models/cube/green_cube.urdf'
BASKET_URDF = '/home/mtb/sim_ws/pybullet_ws/models/basket/basket.urdf'
GREEN_BASKET_URDF = '/home/mtb/sim_ws/pybullet_ws/models/basket_green/basket.urdf'

TABLE_HEIGHT = 0.126

limits = {'x_min': 0.2, 'x_max': 0.35,
          'y_min': -0.3, 'y_max': 0.3}
block_thresh = 0.125

BLOCKS = ["%(color)sBlock%(indice)d" % ({'color': name, 'indice': indice}) for name in ['red'] for
          indice in [1, 2]]
BLOCK_SIZE = 0.05
Block = namedtuple('Block', ['pose', 'handle', 'name'])

I = Interface()

I.connect()

robot = I.load_model(ROBOT_URDF_PATH)
# table = I.load_model(TABLE, Pose(Point(0.4, 0.0, 0.05), Euler(1.57, 0.0, 0.0)),
#                      fixed_base = True, scaling = 1.0)

# I.load_model(CUBE_URDF, Pose(Point(0.3, -0.1, 0.126))
#              ,
#              fixed_base=True, scaling=1.0)
#
#
# I.load_model(GREEN_BASKET_URDF, Pose(Point(0.4, 0.25, 0.126)) ,
#              fixed_base=True, scaling=1.0)
# I.load_model(BASKET_URDF, Pose(Point(0.45, 0.1, 0.126)) ,
#              fixed_base=True, scaling=1.0)
# #
# I.load_model(SPHERE_URDF, Pose(Point(0.2, 0.1, 0.126)) ,
#              fixed_base=True, scaling=0.05)
#
# I.load_model(GSPHERE_URDF, Pose(Point(0.2, 0.2, 0.126)) ,
#              fixed_base=True, scaling=0.05)
#
# I.load_model(GREEN_CUBE_URDF, Pose(Point(0.4, -0.1, 0.126))
#              ,
#              fixed_base=True, scaling=1.0)
#
#
# I.load_model(CYLINDER_URDF, Pose(Point(0.3, 0.1, 0), Euler(1.57,1.57,0)) ,
#              fixed_base=True, scaling=1)
#
# I.load_model(GCYLINDER_URDF, Pose(Point(0.4, 0.1, 0), Euler(1.57,1.57,0)) ,
#              fixed_base=True, scaling=1)


# -------------------G-------------------------------#
# I.load_model(CUBE_URDF, Pose(Point(0.45, 0.1, 0.126)), fixed_base=True, scaling=1.0)
#
#
# I.load_model(GREEN_BASKET_URDF, Pose(Point(0.4, 0.25, 0.126)) ,
#              fixed_base=True, scaling=1.0)
# I.load_model(BASKET_URDF, Pose(Point(0.45, 0.1, 0.126)) ,
#              fixed_base=True, scaling=1.0)
#
#
# I.load_model(GREEN_CUBE_URDF, Pose(Point(0.4, 0.25, 0.126))
#              ,
#              fixed_base=True, scaling=1.0)

# -------------------G-------------------------------#
# I.load_model(CYLINDER_URDF, Pose(Point(0.4, 0.4, 0), Euler(1.57,1.57,0)) ,  fixed_base=True, scaling=1)
# I.load_model(CUBE_URDF, Pose(Point(0.45, 0.06, 0.17)), fixed_base=True, scaling=1.0)
#
# I.load_model(GCYLINDER_URDF, Pose(Point(0.4, 0.25, 0), Euler(1.57,1.57,0)) ,  fixed_base=True, scaling=1)
# I.load_model(GREEN_CUBE_URDF, Pose(Point(0.45, -0.1, 0.17)), fixed_base=True, scaling=1.0)


# #-------------------G-------------------------------#
# I.load_model(SPHERE_URDF, Pose(Point(0.45, 0.1, 0.126)), fixed_base=True, scaling=0.05)
#
#
# I.load_model(GREEN_BASKET_URDF, Pose(Point(0.4, 0.25, 0.126)) ,
#              fixed_base=True, scaling=1.0)
# I.load_model(BASKET_URDF, Pose(Point(0.45, 0.1, 0.126)) ,
#              fixed_base=True, scaling=1.0)
#
#
# I.load_model(GSPHERE_URDF, Pose(Point(0.4, 0.25, 0.126))
#              ,
#              fixed_base=True, scaling=0.05)

# I.load_model(CUBE_URDF, Pose(Point(0.25, 0.1, 0.17)), fixed_base = True, scaling = 1.0)
# I.load_model(CUBE_URDF, Pose(Point(0.25, 0.1, 0.127)), fixed_base = True, scaling = 1.0)

# I.load_model(BASKET_URDF, Pose(Point(0.45, 0.1, 0.126)), fixed_base = True, scaling = 1.0)
#
# I.load_model(GREEN_CUBE_URDF, Pose(Point(0.45, -0.2, 0.126)), fixed_base=True, scaling=1.0)
# I.load_model(GREEN_BASKET_URDF, Pose(Point(0.45, -0.2, 0.126)), fixed_base=True, scaling=1.0)

# I.load_model(GREEN_CUBE_URDF, Pose(Point(0.4, 0.25, 0.17)), fixed_base=True, scaling=1.0)


# I.load_model(CUBE_URDF, Pose(Point(0.45, -0.3, 0.15)), fixed_base = True, scaling = 1.0)
# I.load_model(CUBE_URDF, Pose(Point(0.45, 0.1, 0.126)), fixed_base = True, scaling = 1.0)
#
# I.load_model(BASKET_URDF, Pose(Point(0.45, 0.1, 0.126)), fixed_base = True, scaling = 1.0)
#
# I.load_model(GREEN_CUBE_URDF, Pose(Point(0.45, -0.25, 0.15)),     fixed_base=True, scaling=1.0)
# I.load_model(GREEN_BASKET_URDF, Pose(Point(0.45, -0.25, 0.126)), fixed_base=True, scaling=1.0)
