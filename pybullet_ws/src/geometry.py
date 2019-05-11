import numpy as np
import pybullet as p

def Point(x=0., y=0., z=0.):
    return np.array([x, y, z])

def quat_from_euler(euler):
    return p.getQuaternionFromEuler(euler)

def Euler(roll=0., pitch=0., yaw=0.):
    return np.array([roll, pitch, yaw])

def Pose(point=None, euler=None):
    point = Point() if point is None else point
    euler = Euler() if euler is None else euler
    return (point, quat_from_euler(euler))