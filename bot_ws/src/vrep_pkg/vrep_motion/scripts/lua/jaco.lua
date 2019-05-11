--
-- Created by IntelliJ IDEA.
-- User: mtb
-- Date: 18/08/28
-- Time: 21:10
-- To change this template use File | Settings | File Templates.
--

-- This is a threaded script, and is just an example!

--jointHandles={-1,-1,-1,-1,-1,-1}
--for i=1,6,1 do
--    jointHandles[i]=simGetObjectHandle('Jaco_joint'..i)
--end
--
--- - Set-up some of the RML vectors:
-- vel=35
-- accel=10
-- jerk=5
-- currentVel={0,0,0,0,0,0}
-- currentAccel={0,0,0,0,0,0}
-- maxVel={vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180}
-- maxAccel={accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180}
-- maxJerk={jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180}
-- targetVel={0,0,0,0,0,0}
--
-- targetPos1={90*math.pi/180,90*math.pi/180,90*math.pi/180,90*math.pi/180,90*math.pi/180,90*math.pi/180}
-- simRMLMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos1,targetVel,{1,0,0,-1,-1,-1})
--
-- targetPos2={90*math.pi/180,135*math.pi/180,225*math.pi/180,180*math.pi/180,180*math.pi/180,350*math.pi/180}
-- simRMLMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos2,targetVel,{0,0,0,1,1,1})
--
-- targetPos3={math.pi,math.pi,math.pi,math.pi,math.pi,math.pi}
-- simRMLMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos3,targetVel,{-1,0,0,1,1,1})




-- See the end of the script for instructions on how to do efficient grasping

--if (sim_call_type==sim_childscriptcall_initialization) then
--    modelHandle=simGetObjectAssociatedWithScript(sim_handle_self)
--    j0=simGetObjectHandle("JacoHand_fingers12_motor1")
--    j1=simGetObjectHandle("JacoHand_fingers12_motor2")
--    j2=simGetObjectHandle("JacoHand_finger3_motor1")
--    j3=simGetObjectHandle("JacoHand_finger3_motor2")
--    ui=simGetUIHandle('JacoHand')
--    simSetUIButtonLabel(ui,0,simGetObjectName(modelHandle))
--    closingVel=-0.04
--end
--
--if (sim_call_type==sim_childscriptcall_cleanup) then
--
--end
--
--if (sim_call_type==sim_childscriptcall_actuation) then
--    closing=simBoolAnd16(simGetUIButtonProperty(ui,20),sim_buttonproperty_isdown)~=0
--
--    if (not closing) then
--        simSetJointTargetVelocity(j0,-closingVel)
--        simSetJointTargetVelocity(j1,-closingVel)
--        simSetJointTargetVelocity(j2,-closingVel)
--        simSetJointTargetVelocity(j3,-closingVel)
--    else
--        simSetJointTargetVelocity(j0,closingVel)
--        simSetJointTargetVelocity(j1,closingVel)
--        simSetJointTargetVelocity(j2,closingVel)
--        simSetJointTargetVelocity(j3,closingVel)
--    end
--
--    -- You have basically 2 alternatives to grasp an object:
--    --
--    -- 1. You try to grasp it in a realistic way. This is quite delicate and sometimes requires
--    --    to carefully adjust several parameters (e.g. motor forces/torques/velocities, friction
--    --    coefficients, object masses and inertias)
--    --
--    -- 2. You fake the grasping by attaching the object to the gripper via a connector. This is
--    --    much easier and offers very stable results.
--    --
--    -- Alternative 2 is explained hereafter:
--    --
--    --
--    -- a) In the initialization phase, retrieve some handles:
--    --
--    -- connector=simGetObjectHandle('JacoHand_attachPoint')
--    -- objectSensor=simGetObjectHandle('JacoHand_attachProxSensor')
--
--    -- b) Before closing the gripper, check which dynamically non-static and respondable object is
--    --    in-between the fingers. Then attach the object to the gripper:
--    --
--    -- index=0
--    -- while true do
--    --     shape=simGetObjects(index,sim_object_shape_type)
--    --     if (shape==-1) then
--    --         break
--    --     end
--    --     if (simGetObjectInt32Parameter(shape,sim_shapeintparam_static)==0) and (simGetObjectInt32Parameter(shape,sim_shapeintparam_respondable)~=0) and (simCheckProximitySensor(objectSensor,shape)==1) then
--    --         -- Ok, we found a non-static respondable shape that was detected
--    --         attachedShape=shape
--    --         -- Do the connection:
--    --         simSetObjectParent(attachedShape,connector,true)
--    --         break
--    --     end
--    --     index=index+1
--    -- end
--
--    -- c) And just before opening the gripper again, detach the previously attached shape:
--    --
--    -- simSetObjectParent(attachedShape,-1,true)
--end



--if (sim_call_type==sim_childscriptcall_initialization) then
jointHandles = { -1, -1, -1, -1, -1, -1 }
for i = 1, 6, 1 do
    jointHandles[i] = simGetObjectHandle('Jaco_joint' .. i)
end


vel = 35
accel = 10
jerk = 5
currentVel = { 0, 0, 0, 0, 0, 0 }
currentAccel = { 0, 0, 0, 0, 0, 0 }
maxVel = { vel * math.pi / 180, vel * math.pi / 180, vel * math.pi / 180, vel * math.pi / 180, vel * math.pi / 180, vel * math.pi / 180 }
maxAccel = { accel * math.pi / 180, accel * math.pi / 180, accel * math.pi / 180, accel * math.pi / 180, accel * math.pi / 180, accel * math.pi / 180 }
maxJerk = { jerk * math.pi / 180, jerk * math.pi / 180, jerk * math.pi / 180, jerk * math.pi / 180, jerk * math.pi / 180, jerk * math.pi / 180 }
targetVel = { 0, 0, 0, 0, 0, 0 }

targetPos1 = { 90 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180, 90 * math.pi / 180 }
--    while (simGetSimulationState()~=sim_simulation_advancing_abouttostop) do
simRMLMoveToJointPositions(jointHandles, -1, currentVel, currentAccel, maxVel, maxAccel, maxJerk, targetPos1, targetVel, { 1, 0, 0, -1, -1, -1 })
--    end
targetPos2 = { 60 * math.pi / 180, 60 * math.pi / 180, 60 * math.pi / 180, 60 * math.pi / 180, 60 * math.pi / 180, 60 * math.pi / 180 }
simRMLMoveToJointPositions(jointHandles, -1, currentVel, currentAccel, maxVel, maxAccel, maxJerk, targetPos2, targetVel, { 1, 0, 0, -1, -1, -1 })

targetPos3 = { 0 * math.pi / 180, 0 * math.pi / 180, 0 * math.pi / 180, 0 * math.pi / 180, 0 * math.pi / 180, 0 * math.pi / 180 }
simRMLMoveToJointPositions(jointHandles, -1, currentVel, currentAccel, maxVel, maxAccel, maxJerk, targetPos3, targetVel, { 1, 0, 0, -1, -1, -1 })

--end


--if (sim_call_type==sim_childscriptcall_cleanup) then
--    simStopSimulation()
--end


