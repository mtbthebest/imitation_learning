--
-- Created by IntelliJ IDEA.
-- User: mtb
-- Date: 18/08/26
-- Time: 22:42
-- To change this template use File | Settings | File Templates.
--

-- Do following to activate/deactivate the gripper from another location. For example:
--
-- gripperName='BaxterGripper#1' -- specify the full name. If the full name is "BaxterGripper", specify "BaxterGripper#"
-- simSetIntegerSignal(gripperName..'_close',1) -- close
-- simSetIntegerSignal(gripperName..'_close',0) -- open
--
-- See the end of the script for instructions on how to do efficient grasping

if (sim_call_type==sim_childscriptcall_initialization) then
    gripperHandle=simGetObjectHandle('Baxter_rightGripper')
    objectName=simGetObjectName(gripperHandle)

    motorHandle=simGetObjectHandle('Baxter_rightGripper_closeJoint')
    -- the 'BaxterGripper_centerJoint' is handled by a joint callback script (i.e. custom position controller)
    openedGap=simGetScriptSimulationParameter(sim_handle_self,'openedGap')
    closedGap=simGetScriptSimulationParameter(sim_handle_self,'closedGap')
    interval={0.0362-openedGap,openedGap-closedGap}
    simSetJointInterval(motorHandle,false,interval)
    simSetIntegerSignal(objectName..'_close',1)
end

if (sim_call_type==sim_childscriptcall_cleanup) then

end

if (sim_call_type==sim_childscriptcall_actuation) then
    close=simGetIntegerSignal(objectName..'_close')

    if (close==1) then
        simSetJointTargetVelocity(motorHandle,0.005)
    else
        simSetJointTargetVelocity(motorHandle,-0.005)
    end

    -- You have basically 2 alternatives to grasp an object:
    --
    -- 1. You try to grasp it in a realistic way. This is quite delicate and sometimes requires
    --    to carefully adjust several parameters (e.g. motor forces/torques/velocities, friction
    --    coefficients, object masses and inertias)
    --
    -- 2. You fake the grasping by attaching the object to the gripper via a connector. This is
    --    much easier and offers very stable results.
    --
    -- Alternative 2 is explained hereafter:
    --
    --
    -- a) In the initialization phase, retrieve some handles:
    --
    -- connector=simGetObjectHandle('BaxterGripper_attachPoint')
    -- objectSensor=simGetObjectHandle('BaxterGripper_attachProxSensor')

    -- b) Before closing the gripper, check which dynamically non-static and respondable object is
    --    in-between the fingers. Then attach the object to the gripper:
    --
    -- index=0
    -- while true do
    --     shape=simGetObjects(index,sim_object_shape_type)
    --     if (shape==-1) then
    --         break
    --     end
    --     if (simGetObjectInt32Parameter(shape,sim_shapeintparam_static)==0) and (simGetObjectInt32Parameter(shape,sim_shapeintparam_respondable)~=0) and (simCheckProximitySensor(objectSensor,shape)==1) then
    --         -- Ok, we found a non-static respondable shape that was detected
    --         attachedShape=shape
    --         -- Do the connection:
    --         simSetObjectParent(attachedShape,connector,true)
    --         break
    --     end
    --     index=index+1
    -- end

    -- c) And just before opening the gripper again, detach the previously attached shape:
    --
    -- simSetObjectParent(attachedShape,-1,true)
end


