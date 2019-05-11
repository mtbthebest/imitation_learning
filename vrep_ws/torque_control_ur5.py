
import numpy as np
import vrep
import time
import os
from utils import Csv

filename='data/torques.csv'

#simulation timestep
dt = .005

csv_reader = Csv(filename).read()

data =np.array([map(float,val) for val in csv_reader.values()])

data_times= data[10:,0:]
secs = [data_times[0][i+1] -data_times[0][i] for i in range(200)]
nsecs = [data_times[1][i+1] -data_times[1][i] for i in range(200)]

#Interval of time for time simulation
timestep = np.array(secs) + np.array(nsecs) / 1e09

#Torques values for replay
# torques = [data[:5,i] for i in range(200)]
torques =np.array ([-10.0] + [5.0]*4)

print torques[0].shape



#Velocity of each joint is set to 10000.0 to allow the joint to just use the torque force
velocity = np.ones(5) * 10000.0

#Change the sign of the velocity if we have negative torques
# velocity = np.multiply(velocity,np.sign([data[:6,i] for i in range(200)]))

#The number of times we trigger the simulation step before trying the next torque values
loop = [int(t) +1 for t in (timestep/dt)]

# torques =np.array([10.0])

# close any open connections
vrep.simxFinish(-1)

# Connect to the V-REP continuous server
clientID = vrep.simxStart('127.0.0.1', 19997, True, True, 500, 5)

try:
    if clientID != -1: # if we connected successfully
        print('Connected to remote API server')

        # --------------------- Setup the simulation

        vrep.simxSynchronous(clientID, True)

        #Jaco joint_names
        joint_names = ['UR5_joint%d'%i for i in range(0,5)]

        # Get the handles for each joint
        joint_handles = [vrep.simxGetObjectHandle(
            clientID,
            name,
            vrep.simx_opmode_blocking)[1] for name in joint_names]


        vrep.simxSetFloatingParameter(
            clientID,
            vrep.sim_floatparam_simulation_time_step,
            dt,  # specify a simulation time step
            vrep.simx_opmode_oneshot)

        # strt our simulation in lockstep with our code
        vrep.simxStartSimulation(
            clientID,
            vrep.simx_opmode_blocking)

        k = 0

        # For 300 torques values retrieved from the csv files

        while k<200:
            #Send the torques values
            uu = torques
            print('k = %d'%k)

            for ii, joint_handle in enumerate(joint_handles):
                _, u = vrep.simxGetJointForce(
                    clientID,
                    joint_handle,
                    vrep.simx_opmode_blocking)
                if _ != 0:
                    raise Exception('Error retrieving joint torque, ' +
                                    'return code ', _)
                # print u
                # print uu[ii]
                if np.sign(u) * np.sign(uu[ii]) <= 0:

                    velocity[ii] *= -1

                    _ = vrep.simxSetJointTargetVelocity(
                        clientID,
                        joint_handle,
                        velocity[ii],
                        vrep.simx_opmode_blocking)
                    if _ != 0:
                        raise Exception('Error setting joint target velocity, ' +
                                        'return code ', _)

                # and now modulate the force
                _=vrep.simxSetJointForce(
                    clientID,
                    joint_handle,
                    np.abs(uu[ii]),  # force to apply
                    vrep.simx_opmode_blocking)
                if _ != 0:
                    raise Exception('Error setting max joint force, ' +
                                    'return code ', _)


            # For a required number of time trigger the next simulation step
            # for t in range(loop[k]):
            vrep.simxSynchronousTrigger(clientID)
            # vrep.simxGetPingTime(clientID)
            k +=1
            # time.sleep(1.0)






    else:
        raise Exception('Failed connecting to remote API server')
finally:
    # stop the simulation
    vrep.simxStopSimulation(clientID, vrep.simx_opmode_blocking)

    vrep.simxGetPingTime(clientID)

    # Now close the connection to V-REP:
    vrep.simxFinish(clientID)
    print('connection closed...')



#