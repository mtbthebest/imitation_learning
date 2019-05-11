#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['rbt_baxter_interface', 'rbt_baxter_control', 'rbt_baxter_dataflow',
                 'joint_trajectory_action', 'gripper_action', 'head_action']
d['package_dir'] = {'': 'src'}

setup(**d)
