
import numpy as np
import vrep


filename='/home/mtb/myFileName.txt'


#simulation timestep
dt = .005

with open(filename,'r') as csv_reader:

    data = []
    n=0
    for v in csv_reader:

        a =v.split('\t')
        if len(a) ==8:
            a.pop(-1)
        b = []


        if n>=1:
            for elem in a:
                b.append(float(elem.strip('\n')))
            data.append(b)
        n +=1



data = np.array(data)
timestep = data[:,0]

torques=data[:,1:]

a =[timestep[i+1] -timestep[i] for i in range(timestep.shape[0] -1)]
print max(a)
# #Velocity of each joint is set to 10000.0 to allow the joint to just use the torque force
# velocity = np.ones(6) * 10000.0
# #
# # close any open connections
# vrep.simxFinish(-1)
#
# # Connect to the V-REP continuous server
# clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5)
#
# try:
#     if clientID != -1: # if we connected successfully
#         print('Connected to remote API server')
#
#         # --------------------- Setup the simulation
#
#         vrep.simxSynchronous(clientID, True)
#
#         #Jaco joint_names
#         joint_names = ['UR5_joint%d'%i for i in range(0,6)]
#
#         # Get the handles for each joint
#         joint_handles = [vrep.simxGetObjectHandle(
#             clientID,
#             name,
#             vrep.simx_opmode_blocking)[1] for name in joint_names]
#
#
#         vrep.simxSetFloatingParameter(
#             clientID,
#             vrep.sim_floatparam_simulation_time_step,
#             dt,  # specify a simulation time step
#             vrep.simx_opmode_oneshot)
#
#         # strt our simulation in lockstep with our code
#         vrep.simxStartSimulation(
#             clientID,
#             vrep.simx_opmode_blocking)
#
#         k = 0
#
#         # For 300 torques values retrieved from the csv files
#
#         while k<700:
#             #Send the torques values
#             uu = torques[k]
#             print('k = %d'%k)
#
#             for ii, joint_handle in enumerate(joint_handles):
#                 _, u = vrep.simxGetJointForce(
#                     clientID,
#                     joint_handle,
#                     vrep.simx_opmode_blocking)
#                 if _ != 0:
#                     raise Exception('Error retrieving joint torque, ' +
#                                     'return code ', _)
#                 # print u
#                 # print uu[ii]
#                 if np.sign(u) * np.sign(uu[ii]) <= 0:
#
#                     velocity[ii] *= -1
#
#                     _ = vrep.simxSetJointTargetVelocity(
#                         clientID,
#                         joint_handle,
#                         velocity[ii],
#                         vrep.simx_opmode_blocking)
#                     if _ != 0:
#                         raise Exception('Error setting joint target velocity, ' +
#                                         'return code ', _)
#
#                 # and now modulate the force
#                 _=vrep.simxSetJointForce(
#                     clientID,
#                     joint_handle,
#                     np.abs(uu[ii]),  # force to apply
#                     vrep.simx_opmode_blocking)
#                 if _ != 0:
#                     raise Exception('Error setting max joint force, ' +
#                                     'return code ', _)
#
#
#             # For a required number of time trigger the next simulation step
#             # for t in range(loop[k]):
#             vrep.simxSynchronousTrigger(clientID)
#             # vrep.simxGetPingTime(clientID)
#             k +=1
#             # time.sleep(1.0)
#
#
#
#
#
#
#     else:
#         raise Exception('Failed connecting to remote API server')
# finally:
#     # stop the simulation
#     import time
#     time.sleep(25)
#     vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)
#
#     vrep.simxGetPingTime(clientID)
#
#     # Now close the connection to V-REP:
#     vrep.simxFinish(clientID)
#     print('connection closed...')