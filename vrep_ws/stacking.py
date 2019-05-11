# coding=utf-8

import vrep
import numpy as np

from src.scene_object import SceneObject
import time
import random
from utils import mkdir, Csv

from math import sqrt
import socket



BLOCKS =["%(color)sBlock%(indice)d" % ({'color': name ,'indice':indice})for name in ['red','green','blue'] for indice in [1,2]]

ADDR = '192.168.1.20'
# ADDR = '127.0.0.1'
PORT_NUM = 19997
JOINTS=['Jaco_joint%d'%i for i in range(1,7) ]
TABLE_HEIGHT = 0.18
TRIALS = 10
DEMO_FOLD = '/media/mtb/Data Disk/JACO/DEMO/STACKING/'
SUB_RSLT = ['NEUTRAL_POSE','RED_BLOCK','GREEN_BLOCK','BLUE_BLOCK']
DATA =['img','torques']

limits = {'x_min':0.325,'x_max':0.575,'y_min':-0.275,'y_max':0.275}
block_thresh = 0.15

class PickAndPlace:
    def __init__(self):
        self.sceneobject = SceneObject(ADDR, PORT_NUM)

        # Set the positions of the blocks
        self.blocks = self.set_block_pose()

        self.emptyBuff = bytearray()
        self.sim = self.sceneobject.start_simulation()

        # Robot joints Handle
        self.joints_handle = [self.sceneobject.getobjecthandle(joint,
                                                               operation_mode = vrep.simx_opmode_oneshot_wait) for joint in JOINTS ]

        # Robot IK target Handle
        self.robotTarget = self.sceneobject.getobjecthandle('Jaco_target',
                                                            operation_mode = vrep.simx_opmode_oneshot_wait)
        self.eef = self.sceneobject.getobjecthandle('EEF',
                                                    operation_mode = vrep.simx_opmode_oneshot_wait)

        # Neutral Dummy Handle
        self.neutralDummyHandle = self.sceneobject.getobjecthandle('neutralPose',
                                                                   operation_mode = vrep.simx_opmode_blocking)

        # approach Dummy Handle
        self.approachHandle = self.sceneobject.getobjecthandle('approach')


        # #Image thread
        # img_thread = Thread(target = self.get_img,kwargs = dict(sensor_handle=self.sceneobject.getobjecthandle('Vision_sensor')))
        # img_thread.start()
        #
        # Torque thread
        # torque_thread = Thread(target = self.get_torque)
        # torque_thread.start()
        #
        # # img_thread.join()
        # # torque_thread.join()
        # Move to Neutral Position using OMPL
        self.moveArm(self.neutralDummyHandle, "moveOMPL")


    def set_block_pose(self):
        blocks_dic = {}
        for blocks in BLOCKS:
            while True:
                r_x = limits['x_min'] + (limits['x_max'] - limits['x_min']) * random.random()
                r_y = limits['y_min'] + (limits['y_max'] - limits['y_min']) * random.random()
                collision = False

                for pose in blocks_dic.values():
                    if sqrt((pose[0] - r_x) ** 2 + (pose[1] - r_y) ** 2) <=block_thresh:
                        collision = True
                        break
                if not collision:
                    blocks_dic[blocks] = [r_x,r_y,TABLE_HEIGHT]
                    self.sceneobject.setobjectposition(object_handle = self.sceneobject.getobjecthandle(blocks),relativeTo = -1,position =blocks_dic[blocks] )
                    break

    def moveArm(self, target_handle, planning_mode, operation_mode = vrep.simx_opmode_oneshot_wait):
        target_matrix = None
        while target_matrix is None:
            target_matrix = self.getMatrix(target_handle,vrep.simx_opmode_oneshot_wait)
            if not len(target_matrix) :
                target_matrix = None

        trials = 0
        while trials <3:
            res, retInts, retFloats, retStrings, retBuffer = self.sceneobject.callchildscript('remoteApiServer',
                                                                                          planning_mode, [],
                                                                                          target_matrix, [],
                                                                                          self.emptyBuff,
                                                                                          operation_mode)
            if res and len(retInts):
                if retInts[0] == 1:break
            trials +=1



        if retFloats and retInts:
            self.sceneobject.callchildscript('remoteApiServer',
                                             'run', [],
                                             retFloats, [],
                                             self.emptyBuff,
                                             operation_mode)


            runningPath = True
            
            while runningPath:
                res, retInts, retFloats, retStrings, retBuffer = self.sceneobject.callchildscript('remoteApiServer',
                                                                                             'isrunning', [],
                                                                                             [], [],
                                                                                             self.emptyBuff,
                                                                                             vrep.simx_opmode_oneshot_wait)
                runningPath = retInts[0] == 1


        
        time.sleep(1.0)

    def moveEEF(self, mode, timeout = 2.5, operation_mode = vrep.simx_opmode_blocking):
        res, retInts, retFloats, retStrings, retBuffer = self.sceneobject.callchildscript('remoteApiServer', 'moveEEF',
                                                                                          [mode], [],
                                                                                          [],
                                                                                          self.emptyBuff,
                                                                                          operation_mode)
        time.sleep(timeout)
    
    def place_block(self, block_name, dummy_name, operation_mode = vrep.simx_opmode_oneshot_wait, approach = 'z'):
        block_handle = self.sceneobject.getobjecthandle(block_name, operation_mode)
        dummy_handle = self.sceneobject.getobjecthandle(dummy_name, operation_mode)

        if dummy_handle == None or block_handle is None:
            pass
        else:
        
            self.sceneobject.setobjectposition(dummy_handle, block_handle, [0.0, 0.0, 0.25])
            self.sceneobject.setobjectorientation(dummy_handle, -1, [3.14, 0, 3.14])

            self.sceneobject.setobjectposition(self.approachHandle, block_handle, [-0.03, 0.0, 0.15])
            self.sceneobject.setobjectorientation(dummy_handle, -1, [3.14, 0, 3.14])

            self.moveArm(dummy_handle, "moveIK")
            self.moveArm(self.approachHandle, "moveIK")

            self.moveEEF(mode = 0)
            self.moveArm(dummy_handle, "moveIK")
            self.moveArm(self.neutralDummyHandle, "moveIK")
    
    def pick_block(self, block_name, dummy_name, operation_mode = vrep.simx_opmode_oneshot_wait, approach = 'z'):
        block_handle = self.sceneobject.getobjecthandle(block_name, operation_mode)
        dummy_handle = self.sceneobject.getobjecthandle(dummy_name, operation_mode)
        
        if dummy_handle == None or block_handle is None:
            pass
        else:
            self.sceneobject.setobjectposition(dummy_handle, block_handle, [0.0, 0.0, 0.2])
            self.sceneobject.setobjectorientation(dummy_handle, -1, [3.14, 0, 3.14])
            
            self.sceneobject.setobjectposition(self.approachHandle, block_handle, [-0.015, 0.0, 0.065])
            self.sceneobject.setobjectorientation(dummy_handle, -1, [3.14, 0, 3.14])
            
            self.moveArm(dummy_handle, "moveIK")
            self.moveArm(self.approachHandle, "moveIK")
            self.moveEEF(mode = 1)
            self.moveArm(dummy_handle, "moveIK")
            self.moveArm(self.neutralDummyHandle, "moveIK")
    
    def setPose(self, handle, relativeTo, position, angles, operation_mode = vrep.simx_opmode_oneshot_wait):
        """
        :param handle: handle name
        :param position: x,y,z positions
        :param angles: alpha,beta,gamma orientation values
        :param relativeTo: relative frame
        :return: bool
        """
        self.sceneobject.setobjectpose(handle, relativeTo, position, angles, operation_mode)
    
    def getMatrix(self, handle, operation_mode = vrep.simx_opmode_blocking):
        res, _, matrix, _, _ = self.sceneobject.callchildscript('remoteApiServer', 'getObjectPose', [handle], [], [],
                                                                self.emptyBuff, operation_mode
                                                                )
        return matrix

    def terminate(self):
        self.sceneobject.stop_simulation()

    def get_img(self,sensor_handle):
        print('Getting the image')
        res = 0
        while not res:
            res,_,_ = self.sceneobject.getImage(sensor_handle, 0, vrep.simx_opmode_streaming)
            time.sleep(0.25)
        i=0
        if res:
            while True:
                _,resolution,img = self.sceneobject.getImage(sensor_handle,0)
                i +=1
                if img:
                    image = np.array(img,np.uint8).reshape([resolution[0],resolution[1],-1])
                    np.save('img/img%d.npy'%i,image)


        
def main():
    # for j in range(TRIALS):
        # RSLT = DEMO_FOLD + 'DEMO_%s'%j +'/'
        # for folder in SUB_RSLT:
        #     mkdir(RSLT + folder)
        time.sleep(2.0)
        pick_place = PickAndPlace()


        pick_place.pick_block(block_name = 'redBlock1', dummy_name = 'redBlockDummy1', approach = 'z')
        pick_place.place_block(block_name = 'redBlock2', dummy_name = 'redBlockDummy2', approach = 'z')


        pick_place.pick_block(block_name = 'greenBlock1', dummy_name = 'greenBlockDummy1', approach = 'z')
        pick_place.place_block(block_name = 'greenBlock2', dummy_name = 'greenBlockDummy2', approach = 'z')

        pick_place.pick_block(block_name = 'blueBlock1', dummy_name = 'blueBlockDummy1', approach = 'z')
        pick_place.place_block(block_name = 'blueBlock2', dummy_name = 'blueBlockDummy2', approach = 'z')


        pick_place.terminate()

        



if __name__ == '__main__':
    # print(BLOCKS)
    main()
   #  thread1 = Thread(target = test1)
   #  thread2 = Thread(target = test2)
   #  thread1.start()
   #  thread2.start()
   #  thread1.join()
   #  thread2.join()
   #  a = np.array([3.0,4.0])
   #  b = np.where(2<np.all(a)<5,[True],[False])
   #  c = np.all(b)
   #  print(b)
   #  print(c)







