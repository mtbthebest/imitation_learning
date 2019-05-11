displayInfo=function(txt)
    if dlgHandle then
        simEndDialog(dlgHandle)
    end
    dlgHandle=nil
    if txt and #txt>0 then
        dlgHandle=simDisplayDialog('info',txt,sim_dlgstyle_message,false)
        simSwitchThread()
    end
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

getShiftedMatrix=function(matrix,localShift,dir)
    -- Returns a pose or matrix shifted by vector localShift
    local m={}
    for i=1,12,1 do
        m[i]=matrix[i]
    end
    m[4]=m[4]+dir*(m[1]*localShift[1]+m[2]*localShift[2]+m[3]*localShift[3])
    m[8]=m[8]+dir*(m[5]*localShift[1]+m[6]*localShift[2]+m[7]*localShift[3])
    m[12]=m[12]+dir*(m[9]*localShift[1]+m[10]*localShift[2]+m[11]*localShift[3])
    return m
end

getConfig=function()
    -- Returns the current robot configuration
    local config={}
    for i=1,#jh,1 do
        config[i]=simGetJointPosition(jh[i])
    end
    return config
end

setConfig=function(config)
    -- Applies the specified configuration to the robot
    if config then
        for i=1,#jh,1 do
            simSetJointPosition(jh[i],config[i])
        end
    end
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

getPathLength=function(path)
    -- Returns the length of the path in configuration space
    local d=0
    local l=#jh
    local pc=#path/l
    for i=1,pc-1,1 do
        local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6],path[(i-1)*l+7]}
        local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6],path[i*l+7]}
        d=d+getConfigConfigDistance(config1,config2)
    end
    return d
end

findPath=function(startConfig,goalConfigs,cnt)
    -- Here we do path planning between the specified start and goal configurations. We run the search cnt times,
    -- and return the shortest path, and its length

    -- Following because the robot has "strange" joint limits, e.g. +-10'000, and searching such a large
    -- space would be inefficient for path planning
    local task=simExtOMPL_createTask('task')
    simExtOMPL_setAlgorithm(task,sim_ompl_algorithm_RRTConnect)
    local j1_space=simExtOMPL_createStateSpace('j1_space',sim_ompl_statespacetype_joint_position,jh[1],{-170*math.pi/180},{170*math.pi/180},1)
    local j2_space=simExtOMPL_createStateSpace('j2_space',sim_ompl_statespacetype_joint_position,jh[2],{-120*math.pi/180},{120*math.pi/180},2)
    local j3_space=simExtOMPL_createStateSpace('j3_space',sim_ompl_statespacetype_joint_position,jh[3],{-170*math.pi/180},{170*math.pi/180},3)
    local j4_space=simExtOMPL_createStateSpace('j4_space',sim_ompl_statespacetype_joint_position,jh[4],{-120*math.pi/180},{120*math.pi/180},0)
    local j5_space=simExtOMPL_createStateSpace('j5_space',sim_ompl_statespacetype_joint_position,jh[5],{-170*math.pi/180},{170*math.pi/180},0)
    local j6_space=simExtOMPL_createStateSpace('j6_space',sim_ompl_statespacetype_joint_position,jh[6],{-120*math.pi/180},{120*math.pi/180},0)
    local j7_space=simExtOMPL_createStateSpace('j7_space',sim_ompl_statespacetype_joint_position,jh[7],{-170*math.pi/180},{170*math.pi/180},0)
    simExtOMPL_setStateSpace(task,{j1_space,j2_space,j3_space,j4_space,j5_space,j6_space,j7_space})
    simExtOMPL_setCollisionPairs(task,collisionPairs)
    simExtOMPL_setStartState(task,startConfig)
    simExtOMPL_setGoalState(task,goalConfigs[1])
    for i=2,#goalConfigs,1 do
        simExtOMPL_addGoalState(task,goalConfigs[i])
    end
    local path=nil
    local l=999999999999
    --    forbidThreadSwitches(true)
    for i=1,cnt,1 do
        local res,_path=simExtOMPL_compute(task,maxOMPLCalculationTime,-1,300)
        if res>0 and _path then
            local _l=getPathLength(_path)
            if _l<l then
                l=_l
                path=_path
            end
        end
--        if path then
--            visualizePath(path)
--        end
    end
    --    forbidThreadSwitches(false)
    simExtOMPL_destroyTask(task)
    return path,l
end

findShortestPath=function(startConfig,goalConfigs,searchCntPerGoalConfig)
    -- This function will search for several paths between the specified start configuration,
    -- and several of the specified goal configurations. The shortest path will be returned
    local onePath,onePathLength=findPath(startConfig,goalConfigs,searchCntPerGoalConfig)
    return onePath,generatePathLengths(onePath)
end

generateIkPath=function(startConfig,goalPose,steps)
    -- Generates (if possible) a linear, collision free path between a robot config and a target pose
    forbidThreadSwitches(true)
    local currentConfig=getConfig()
    setConfig(startConfig)
    simSetObjectMatrix(ikTarget,-1,goalPose)
    local c=simGenerateIkPath(ikGroup,jh,steps,collisionPairs)
    setConfig(currentConfig)
    forbidThreadSwitches(false)
    return c
end

forbidThreadSwitches=function(forbid)
    -- Allows or forbids automatic thread switches.
    -- This can be important for threaded scripts. For instance,
    -- you do not want a switch to happen while you have temporarily
    -- modified the robot configuration, since you would then see
    -- that change in the scene display.
    if forbid then
        forbidLevel=forbidLevel+1
        if forbidLevel==1 then
            simSetThreadAutomaticSwitch(false)
        end
    else
        forbidLevel=forbidLevel-1
        if forbidLevel==0 then
            simSetThreadAutomaticSwitch(true)
        end
    end
end

generatePathLengths=function(path)
    -- Returns the length of the path in configuration space
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

findCollisionFreeConfig=function(matrix)
    -- Here we search for a robot configuration..
    -- 1. ..that matches the desired pose (matrix)
    -- 2. ..that does not collide in that configuration
    simSetObjectMatrix(ikTarget,-1,matrix)

    -- This robot has 4 joints that have a huge range (i.e. -10'000 - +10'000 degrees)
    -- And since we do not want to search that huge space, we limit the range around the current configuration
    -- We actually do the same during path search
    local c=simGetConfigForTipPose(ikGroup,jh,0.65,10,nil,collisionPairs)
--    if c then
--        -- Here we check point 3:
--        local m=matrix
--        local path=generateIkPath(c,m,ikSteps)
--        if path==nil then
--            c=nil
--        end
--    end
    return c

end

findSeveralCollisionFreeConfigs=function(matrix,trialCnt,maxConfigs)
    -- Here we search for several robot configurations...
    -- 1. ..that matches the desired pose (matrix)
    -- 2. ..that does not collide in that configuration
    -- 3. ..that does not collide and that can perform the IK linear approach
    simSetObjectMatrix(ikTarget,-1,matrix)
    local cc=getConfig()
    local cs={}
    local l={}
    for i=1,trialCnt,1 do
        local c=findCollisionFreeConfig(matrix)
        if c then
            local dist=getConfigConfigDistance(cc,c)
            local p=0
            local same=false
            for j=1,#l,1 do
                if math.abs(l[j]-dist)<0.001 then
                    -- we might have the exact same config. Avoid that
                    same=true
                    for k=1,#jh,1 do
                        if math.abs(cs[j][k]-c[k])>0.01 then
                            same=false
                            break
                        end
                    end
                end
                if same then
                    break
                end
            end
            if not same then
                cs[#cs+1]=c
                l[#l+1]=dist
            end
        end
        if #l>=maxConfigs then
            break
        end
    end
    if #cs==0 then
        cs=nil
    end
    return cs
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



threadFunction=function()
    local m=getShiftedMatrix(simGetObjectMatrix(target1,-1),{0,0,0.1},-1)
    displayInfo('searching for several valid goal configurations...')
    local configs=findSeveralCollisionFreeConfigs(m,300,5)
    displayInfo('searching for several valid paths between the current configuration and found goal configurations...')
    path,lengths=findShortestPath(getConfig(),configs,numberOfOMPLCalculationsPasses)

    if path then
        simAddStatusbarMessage('path')
    end

    executeMotion(path,lengths,maxVel,maxAccel,maxJerk)




end

-- Initialization phase:

jh={-1,-1,-1,-1,-1,-1,-1}
jt={-1,-1,-1,-1,-1,-1,-1}
for i=1,7,1 do
    jh[i]=simGetObjectHandle('Baxter_leftArm_joint'..i)
    jt[i]=simGetJointType(jh[i])
end
baxterBaseHandle=simGetObjectHandle('Baxter')
ikGroup=simGetIkGroupHandle('Baxter_leftArm')
ikTarget=simGetObjectHandle('Baxter_leftArm_target')
ikTip=simGetObjectHandle('Baxter_leftArm_tip')
collisionPairs={simGetCollectionHandle('Baxter_leftArm'),simGetCollectionHandle('Baxter_leftArm')}
target1=simGetObjectHandle('Baxter_leftArm_mpTarget1')

--approachDirectionObstacle=simGetObjectHandle('approachDirectionObstacle')
maxVel=1
maxAccel=1
maxJerk=8000
metric={1,1,1,1,1,1,1}
forbidLevel=0
ikSteps=50

maxOMPLCalculationTime=4 -- for one calculation. Higher is better, but takes more time
OMPLAlgo=sim_ompl_algorithm_BKPIECE1 -- the OMPL algorithm to use
numberOfOMPLCalculationsPasses=4 -- the number of OMPL calculation runs for a same goal config. The more, the better results, but slower

res,err=xpcall(threadFunction,function(err) return debug.traceback(err) end)
if not res then
    simAddStatusbarMessage('Lua runtime error: '..err)
end

