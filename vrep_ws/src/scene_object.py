# coding=utf-8

import vrep
import time
import sys


class VREP(object):
    def __init__(self, addr, port):
        self.addr = addr
        self.port = port
        self.timeout = 5
        
        vrep.simxFinish(-1)
        self.clientID = vrep.simxStart(self.addr, self.port, True, True, self.timeout, 5)
        if self.clientID > -1:
            print("Connection successful")
        else:
            return Exception("Not connected to the server API")
    
    def disconnect(self):
        if hasattr(self, 'clientID'):
            vrep.simxFinish(self.clientID)
        else:
            vrep.simxFinish(-1)
    
    def start_simulation(self, sync = False):
        """Return responseflag"""
        if sync: self.enablesync()
        start = time.time()
        
        while (time.time() - start < self.timeout):
            response = vrep.simxStartSimulation(self.clientID, vrep.simx_opmode_oneshot_wait)
            if (response == 0):
                print('Simulation started')
                return True
        
        raise Exception('Simulation could not start in the selected timeout')
    
    def stop_simulation(self):
        """Stop the simulation"""
        start = time.time()
        
        if not hasattr(self, 'clientID'):
            raise Exception('The simulation has not been started. Check start_simulation function')
        
        while time.time() - start < self.timeout:
            response = vrep.simxStopSimulation(self.clientID, vrep.simx_opmode_blocking)
            if response == 0:
                print('Simulation stopped')
                vrep.simxFinish(-1)
                return True
        
        raise Exception("Can't stop the simulation")
    
    def getobjecthandle(self, object_name, operation_mode = vrep.simx_opmode_blocking):
        start = time.time()
        
        while (time.time() - start <= self.timeout):
            error_flag, object_handle = vrep.simxGetObjectHandle(self.clientID, object_name, operation_mode)
            if error_flag == vrep.simx_return_ok:
                return object_handle
        
        return None
    
    def getobjectposition(self, object_handle, relativeTo = -1, operation_mode = vrep.simx_opmode_blocking):
        start = time.time()
        while (time.time() - start <= self.timeout):
            error_flag, position = vrep.simxGetObjectPosition(self.clientID, object_handle, relativeTo, operation_mode)
            if error_flag == vrep.simx_return_ok:
                return position
        
        return None
    
    def getobjectorientation(self, object_handle, relativeTo = -1, operation_mode = vrep.simx_opmode_blocking,
                             units = 'rad'):
        start = time.time()
        while (time.time() - start <= self.timeout):
            error_flag, orientation = vrep.simxGetObjectOrientation(self.clientID, object_handle, relativeTo,
                                                                    operation_mode)
            if error_flag == vrep.simx_return_ok:
                if units == 'deg': orientation = map(lambda x: 180.0 * x / 3.1415927410125732, orientation)
                return orientation
        
        return None
    
    def setobjectposition(self, object_handle, relativeTo = -1, position = None,
                          operation_mode = vrep.simx_opmode_blocking):
        if position is None:
            raise Exception('The position values are not set')
        start = time.time()
        while (time.time() - start <= self.timeout):
            error_flag = vrep.simxSetObjectPosition(self.clientID, object_handle, relativeTo, position,
                                                    operation_mode)
            if error_flag == vrep.simx_return_ok:
                return True
        
        return None
    
    def setobjectorientation(self, object_handle, relativeTo = -1, angles = None,
                             operation_mode = vrep.simx_opmode_blocking):
        if angles is None:
            raise Exception('The angles values are not set')
        start = time.time()
        while (time.time() - start <= self.timeout):
            error_flag = vrep.simxSetObjectOrientation(self.clientID, object_handle, relativeTo, angles,
                                                       operation_mode)
            if error_flag == vrep.simx_return_ok:
                return True
        
        return None
    
    def setobjectpose(self, object_handle, relativeTo = -1, position = None, angles = None,
                      operation_mode = vrep.simx_opmode_blocking):
        if angles is None or position is None:
            raise Exception('No values set')
        start = time.time()
        while (time.time() - start <= self.timeout):
            if self.setobjectposition(object_handle, relativeTo, position, operation_mode):
                if self.setobjectorientation(object_handle, relativeTo, angles, operation_mode):
                    return True
        return None
    
    def callchildscript(self, script_name, func_name, inInts, inFloats, inStrings, inBuffer,
                        operation_mode = vrep.simx_opmode_oneshot_wait):
        res, retInts, retFloats, retStrings, retBuffer = vrep.simxCallScriptFunction(self.clientID, script_name,
                                                                                     vrep.sim_scripttype_childscript,
                                                                                     func_name,
                                                                                     inInts, inFloats, inStrings,
                                                                                     inBuffer,
                                                                                     operation_mode)
        return [res, retInts, retFloats, retStrings, retBuffer]
    
    def enablesync(self):
        vrep.simxSynchronous(self.clientID, True)
    
    def triggernextstep(self):
        vrep.simxSynchronousTrigger(self.clientID)
    
    def getImage(self, sensor_handle, color, operation_mode = vrep.simx_opmode_buffer):
        try:
            err_flag, resolution, image = vrep.simxGetVisionSensorImage(self.clientID, sensor_handle, color, operation_mode)
        except MemoryError:
            err_flag, resolution, image = -1, [], []


        return err_flag,resolution, image

    def getTorque(self,joints_handle,operation_mode = vrep.simx_opmode_buffer):
        joints_torque = []
        for joint in joints_handle:
            err_flag, torque = vrep.simxGetJointForce(self.clientID, joint, operation_mode)
            if err_flag ==-1:
                return []
            joints_torque.append(torque)
        
        return joints_torque

class SceneObject(VREP):
    def __init__(self, addr, port):
        super(SceneObject, self).__init__(addr, port)

