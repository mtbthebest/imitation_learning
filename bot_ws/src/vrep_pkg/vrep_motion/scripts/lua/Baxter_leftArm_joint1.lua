--
-- Created by IntelliJ IDEA.
-- User: mtb
-- Date: 18/08/26
-- Time: 22:26
-- To change this template use File | Settings | File Templates.
--
getShiftedMatrix=function(matrix,shift,dir,absoluteShift)
    -- Returns a pose or matrix shifted by vector shift
    local m={}
    for i=1,12,1 do
        m[i]=matrix[i]
    end
    if absoluteShift then
        m[4]=m[4]+dir*shift[1]
        m[8]=m[8]+dir*shift[2]
        m[12]=m[12]+dir*shift[3]
    else
        m[4]=m[4]+dir*(m[1]*shift[1]+m[2]*shift[2]+m[3]*shift[3])
        m[8]=m[8]+dir*(m[5]*shift[1]+m[6]*shift[2]+m[7]*shift[3])
        m[12]=m[12]+dir*(m[9]*shift[1]+m[10]*shift[2]+m[11]*shift[3])
    end
    return m
end

_getJointPosDifference=function(startValue,goalValue,isRevolute)
    local dx=goalValue-startValue
    if (isRevolute) then
        if (dx>=0) then
            dx=math.mod(dx+math.pi,2*math.pi)-math.pi
        else
            dx=math.mod(dx-math.pi,2*math.pi)+math.pi
        end
    end
    return(dx)
end

_applyJoints=function(jointHandles,joints)
    for i=1,#jointHandles,1 do
        simSetJointTargetPosition(jointHandles[i],joints[i])
    end
end

getConfig=function()
    -- Returns the current robot configuration
    local config={}
    for i=1,#jh,1 do
        config[i]=simGetJointPosition(jh[i])
    end
    return config
end

getConfigConfigDistance=function(config1,config2)
    -- Returns the distance (in configuration space) between two configurations
    local d=0
    for i=1,#jh,1 do
        local dx=(config1[i]-config2[i])*metric[i]
        d=d+dx*dx
    end
    return math.sqrt(d)
end

generatePathLengths=function(path)
    -- Returns a table that contains a distance along the path for each path point
    local d=0
    local l=#jh
    local pc=#path/l
    local retLengths={0}
    for i=1,pc-1,1 do
        local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6],path[(i-1)*l+7]}
        local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6],path[i*l+7]}
        d=d+getConfigConfigDistance(config1,config2)
        retLengths[i+1]=d
    end
    return retLengths
end

generateDirectPath=function(goalConfig,steps)
    local startConfig=getConfig()
    local dx={}
    for i=1,#jh,1 do
        dx[i]=(goalConfig[i]-startConfig[i])/(steps-1)
    end
    local path={}
    for i=1,steps,1 do
        for j=1,#jh,1 do
            path[#path+1]=startConfig[j]+dx[j]*(i-1)
        end
    end
    return path, generatePathLengths(path)
end

generateIkPath=function(goalPose,steps)
    -- Generates (if possible) a linear path between a robot config and a target pose
    simSetObjectMatrix(ikTarget,-1,goalPose)
    local c=simGenerateIkPath(ikGroup,jh,steps)
    if c then
        return c, generatePathLengths(c)
    end
end

executeMotion=function(path,lengths,maxVel,maxAccel,maxJerk)
    dt=simGetSimulationTimeStep()

    -- 1. Make sure we are not going too fast for each individual joint (i.e. calculate a correction factor (velCorrection)):
    jointsUpperVelocityLimits={}
    for j=1,7,1 do
        res,jointsUpperVelocityLimits[j]=simGetObjectFloatParameter(jh[j],sim_jointfloatparam_upper_limit)
    end
    velCorrection=1

    simSetThreadAutomaticSwitch(false)
    while true do
        posVelAccel={0,0,0}
        targetPosVel={lengths[#lengths],0}
        pos=0
        res=0
        previousQ={path[1],path[2],path[3],path[4],path[5],path[6],path[7]}
        local rMax=0
        rmlHandle=simRMLPos(1,0.0001,-1,posVelAccel,{maxVel*velCorrection,maxAccel,maxJerk},{1},targetPosVel)
        while res==0 do
            res,posVelAccel,sync=simRMLStep(rmlHandle,dt)
            if (res>=0) then
                l=posVelAccel[1]
                for i=1,#lengths-1,1 do
                    l1=lengths[i]
                    l2=lengths[i+1]
                    if (l>=l1)and(l<=l2) then
                        t=(l-l1)/(l2-l1)
                        for j=1,7,1 do
                            q=path[7*(i-1)+j]+_getJointPosDifference(path[7*(i-1)+j],path[7*i+j],jt[j]==sim_joint_revolute_subtype)*t
                            dq=_getJointPosDifference(previousQ[j],q,jt[j]==sim_joint_revolute_subtype)
                            previousQ[j]=q
                            r=math.abs(dq/dt)/jointsUpperVelocityLimits[j]
                            if (r>rMax) then
                                rMax=r
                            end
                        end
                        break
                    end
                end
            end
        end
        simRMLRemove(rmlHandle)
        if rMax>1.001 then
            velCorrection=velCorrection/rMax
        else
            break
        end
    end
    simSetThreadAutomaticSwitch(true)

    -- 2. Execute the movement:
    posVelAccel={0,0,0}
    targetPosVel={lengths[#lengths],0}
    pos=0
    res=0
    jointPos={}
    rmlHandle=simRMLPos(1,0.0001,-1,posVelAccel,{maxVel*velCorrection,maxAccel,maxJerk},{1},targetPosVel)
    while res==0 do
        dt=simGetSimulationTimeStep()
        res,posVelAccel,sync=simRMLStep(rmlHandle,dt)
        if (res>=0) then
            l=posVelAccel[1]
            for i=1,#lengths-1,1 do
                l1=lengths[i]
                l2=lengths[i+1]
                if (l>=l1)and(l<=l2) then
                    t=(l-l1)/(l2-l1)
                    for j=1,7,1 do
                        jointPos[j]=path[7*(i-1)+j]+_getJointPosDifference(path[7*(i-1)+j],path[7*i+j],jt[j]==sim_joint_revolute_subtype)*t
                    end
                    _applyJoints(jh,jointPos)
                    break
                end
            end
        end
        simSwitchThread()
    end
    simRMLRemove(rmlHandle)
end

setStageAndWaitForNext=function(stageNumber)
    simSetIntegerSignal(signalName,stageNumber)
    while true do
        stage=simGetIntegerSignal(signalName)
        if (stage==stageNumber+1) then
            break
        end
        simSwitchThread() -- don't waste CPU time
    end
end

jh={-1,-1,-1,-1,-1,-1,-1}
jt={-1,-1,-1,-1,-1,-1,-1}
for i=1,7,1 do
    jh[i]=simGetObjectHandle('Baxter_leftArm_joint'..i)
    jt[i]=simGetJointType(jh[i])
end
baseHandle=simGetObjectAssociatedWithScript(sim_handle_self)
baxterBaseHandle=simGetObjectHandle('Baxter')
ikGroup=simGetIkGroupHandle('Baxter_leftArm')
ikTarget=simGetObjectHandle('Baxter_leftArm_target')
target1=simGetObjectHandle('Baxter_leftArm_mpTarget1')
name=simGetObjectName(baseHandle)
suffix=simGetNameSuffix(name)
--simAuxiliaryConsolePrint(handle_console,suffix)
signalName='BaxterLeftArmSignal'
if (suffix>=0) then
    signalName=signalName..'#'..suffix
end

maxVel=3
maxAccel=1
maxJerk=8000
ikSteps=20
fkSteps=160
metric={1,1,1,1,1,1,1}

-- Wait until the monitor told us to continue:
setStageAndWaitForNext(0)

-- Move to the first location (by generating a direct path):
config={90*math.pi/180,-90*math.pi/180,-160*math.pi/180,-30*math.pi/180,-160*math.pi/180,-90*math.pi/180,160*math.pi/180}
path,lengths=generateDirectPath(config,fkSteps)
executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- Move to the second location (by generating a direct path):
--config={-60*math.pi/180,60*math.pi/180,160*math.pi/180,150*math.pi/180,160*math.pi/180,90*math.pi/180,-160*math.pi/180}
--path,lengths=generateDirectPath(config,fkSteps)
--executeMotion(path,lengths,maxVel,maxAccel,maxJerk)


-- Move to the third location (by generating a direct path):
--config={0,0,0,0,0,0,0}
--path,lengths=generateDirectPath(config,fkSteps)
--executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- Move to the forth location (by generating a direct path):
--config={34.61*math.pi/180,12.3*math.pi/180,-84.7*math.pi/180,116.29*math.pi/180,13.94*math.pi/180,52.84*math.pi/180,89.03*math.pi/180}
--path,lengths=generateDirectPath(config,fkSteps)
--executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- Wait until the monitor told us to continue:
--setStageAndWaitForNext(2)

-- Move towards the other arm (with IK):
--m=getShiftedMatrix(simGetObjectMatrix(target1,-1),{0,-0.15,0},1,true)
--path,lengths=generateIkPath(m,ikSteps)
--executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- Wait until the monitor told us to continue:
--setStageAndWaitForNext(4)

-- Move parallel with the other arm (with IK):
--m=getShiftedMatrix(simGetObjectMatrix(target1,-1),{0,0.15,0},1,true)
--path,lengths=generateIkPath(m,ikSteps)
--executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- Wait until the monitor told us to continue:
--setStageAndWaitForNext(6)

-- Move parallel with the other arm (with IK):
--m=getShiftedMatrix(simGetObjectMatrix(target1,-1),{0,-0.45,0},1,true)
--path,lengths=generateIkPath(m,ikSteps)
--executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- Wait until the monitor told us to continue:
--setStageAndWaitForNext(8)

-- Move parallel with the other arm (with IK):
--m=getShiftedMatrix(simGetObjectMatrix(target1,-1),{0,-0.15,0},1,true)
--path,lengths=generateIkPath(m,ikSteps)
--executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- Wait until the monitor told us to continue:
--setStageAndWaitForNext(10)

-- Move away from the other arm (with IK):
--m=getShiftedMatrix(simGetObjectMatrix(target1,-1),{0,0,0},1,true)
--path,lengths=generateIkPath(m,ikSteps)
--executeMotion(path,lengths,maxVel,maxAccel,maxJerk)

-- Move to the neutral location (by generating a direct path):
--config={0,0,0,0,0,0,0}
--path,lengths=generateDirectPath(config,fkSteps)
--executeMotion(path,lengths,maxVel,maxAccel,maxJerk)