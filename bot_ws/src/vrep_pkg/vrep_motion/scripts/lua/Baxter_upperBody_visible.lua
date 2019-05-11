--
-- Created by IntelliJ IDEA.
-- User: mtb
-- Date: 18/08/26
-- Time: 22:29
-- To change this template use File | Settings | File Templates.
--
waitForLeftArm=function(waitNumber,nextNumber)

    while true do
        stage=simGetIntegerSignal(leftArmSignalName)
        if (stage==waitNumber) then
            break
        end
        simSwitchThread() -- don't waste CPU time
    end
    simSetIntegerSignal(leftArmSignalName,nextNumber)
end

--waitForRightArm=function(waitNumber,nextNumber)
--    while true do
--        stage=simGetIntegerSignal(rightArmSignalName)
--        if (stage==waitNumber) then
--            break
--        end
--        simSwitchThread() -- don't waste CPU time
--    end
--    simSetIntegerSignal(rightArmSignalName,nextNumber)
--end

--                %Left%
monitorJointHandle=simGetObjectHandle('Baxter_monitorJoint')
name=simGetObjectName(monitorJointHandle)
suffix=simGetNameSuffix(name)

leftArmSignalName='BaxterLeftArmSignal'
if (suffix>=0) then
    leftArmSignalName=leftArmSignalName..'#'..suffix
end

--rightArmSignalName='BaxterRightArmSignal'
--if (suffix>=0) then
--    rightArmSignalName=rightArmSignalName..'#'..suffix
--end

-- Tell the left arm to start movement:
waitForLeftArm(0,1)

-- Move the head towards the left arm:
simSetJointTargetPosition(monitorJointHandle,30*math.pi/180)

-- Wait 7 seconds:
--simWait(7)

-- Tell the right arm to start movement:
--waitForRightArm(0,1)

-- Move the head towards the right arm:
--simSetJointTargetPosition(monitorJointHandle,-30*math.pi/180)

-- Wait 5 seconds:
--simWait(5)

-- Wait until the left arm is done for the first part:
--waitForLeftArm(2,2)

-- Move the head towards the center:
--simSetJointTargetPosition(monitorJointHandle,0*math.pi/180)

-- Wait until the right arm is done for the first part:
--waitForRightArm(2,2)

-- Now tell the two arms to move close to each other:
--waitForLeftArm(2,3)
--waitForRightArm(2,3)

-- Now tell the two arms to do the next movement:
--waitForLeftArm(4,4)
--waitForRightArm(4,4)
--waitForLeftArm(4,5)
--waitForRightArm(4,5)

-- Now tell the two arms to do the next movement:
--waitForLeftArm(6,6)
--waitForRightArm(6,6)
--waitForLeftArm(6,7)
--waitForRightArm(6,7)

-- Now tell the two arms to do the next movement:
--waitForLeftArm(8,8)
--waitForRightArm(8,8)
--waitForLeftArm(8,9)
--waitForRightArm(8,9)

-- Now tell the two arms to do the next movement:
--waitForLeftArm(10,10)
--waitForRightArm(10,10)
--waitForLeftArm(10,11)
--waitForRightArm(10,11)


