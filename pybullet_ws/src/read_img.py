# coding=utf-8

import scipy.misc
from imageio import imwrite
import numpy as np
import os
from utils import pathname, get_files

PATH = '/home/mtb/sim_ws/pybullet_ws/data/Trial_950/IMG'

# for i,p in enumerate(os.listdir(path)):
#     rgb = np.
# rgb = rgb.reshape((-1,128,128,4))
#
# print rgb.shape
# print np.load('/media/mtb/76f4874c-25bd-4dd1-b7a5-dc5732ba8511/mtb/THROW/Trial_0/velocities.npy').shape


for i in range(1500):
    rgb = np.load(pathname(PATH, 'img_%d.npy' % i))
    scipy.misc.imsave('/home/mtb/IMG/img_%d.jpg' % i, rgb[:, :, :3])
