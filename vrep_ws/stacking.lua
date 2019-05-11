forbidThreadSwitches=function(forbid,forbidLevel)
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
    return forbidLevel
end

visualizePath=function(task,path)
    if not _lineContainer then
        _lineContainer=simAddDrawingObject(sim_drawing_lines,3,0,-1,99999,{1,1,0})
    end
    simAddDrawingObjectItem(_lineContainer,nil)
    if path then
        task.forbidLevel = forbidThreadSwitches(true,task.forbidLevel)
        local initConfig=task.currentState
        local l=#task.jh
        local pc=#path/l
        for i=1,pc-1,1 do
            local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6]}
            local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6]}
            setConfig(task.jh,config1)
            local lineDat=simGetObjectPosition(task.ikTip,-1)
            setConfig(task.jh,config2)
            local p=simGetObjectPosition(task.ikTip,-1)
            lineDat[4]=p[1]
            lineDat[5]=p[2]
            lineDat[6]=p[3]
            simAddDrawingObjectItem(_lineContainer,lineDat)
        end
        setConfig(initConfig)
        task.forbidLevel = forbidThreadSwitches(false,task.forbidLevel)
    end
    simSwitchThread()
end

findCollisionFreeConfig=function(task)
    -- Here we search for a robot configuration..
    -- 1. ..that matches the desired pose (matrix)
    -- 2. ..that does not collide in that configuration
    simSetObjectMatrix(task.ikTarget,-1,task.goalPose)

    -- This robot has 4 joints that have a huge range (i.e. -10'000 - +10'000 degrees)
    -- And since we do not want to search that huge space, we limit the range around the current configuration
    -- We actually do the same during path search
    local cc=getConfig(task.jh)
    local jointLimitsL={}
    local jointRanges={}
    for i=1,#task.jh,1 do
        jointLimitsL[i]=cc[i]-360*math.pi/180
        if jointLimitsL[i]<-10000 then jointLimitsL[i]=-10000 end
        jointRanges[i]=720*math.pi/180
        if cc[i]+jointRanges[i]>10000 then jointRanges[i]=10000-cc[i] end
    end
    jointLimitsL[2]=47*math.pi/180
    jointRanges[2]=266*math.pi/180
    jointLimitsL[3]=19*math.pi/180
    jointRanges[3]=322*math.pi/180
    local c=simGetConfigForTipPose(task.ikGroup,task.jh,0.65,10,nil,task.collisionPairs,nil,jointLimitsL,jointRanges)
    return c
end



findSeveralCollisionFreeConfigs=function(task)
    -- Here we search for several robot configurations...
    -- 1. ..that matches the desired pose (matrix)
    -- 2. ..that does not collide in that configuration
    -- 3. ..that does not collide and that can perform the IK linear approach
    simSetObjectMatrix(task.ikTarget,-1,task.goalPose)

    local cs={}
    local l={}
    for i=1,task.trialCnt,1 do
        local c=findCollisionFreeConfig(task)
        if c then
            local dist=getConfigConfigDistance(task.currentState,c,task.metric)
            local p=0
            local same=false
            for j=1,#l,1 do
                if math.abs(l[j]-dist)<0.001 then
                    -- we might have the exact same config. Avoid that
                    same=true
                    for k=1,#task.jh,1 do
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
        if #l>=task.maxConfigs then
            break
        end
    end
    if #cs==0 then
        cs=nil
    end
    return cs
end



moveToDummy=function(inInts,inFloats,inStrings,inBuffer)
    local task={}
    task.robotHandle=inInts[1]

    local collisionChecking=inInts[2]>0
    task.maxVel=inInts[3]
    task.maxAccel=inInts[4]
    task.maxJerk=inInts[5]
    task.forbidLevel=inInts[6]
    task.metric={0.2,1,0.8,0.1,0.1,0.1}
    task.ikSteps=inInts[7]
    task.trialCnt=inInts[8]
    task.maxConfigs=inInts[9]
    task.numberOfOMPLCalculationsPasses=inInts[10]
    task.maxOMPLCalculationTime=inInts[11]



    local currentState={}
    for i=1,6,1 do currentState[i]=inFloats[i] end
    task.currentState=currentState
    local goalPose={}
    for i=1,12,1 do goalPose[i]=inFloats[i+6] end
    task.goalPose=goalPose



    jh={-1,-1,-1,-1,-1,-1}
    jt={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        jh[i]=simGetObjectHandle('Jaco_joint'..i)
        jt[i]=simGetJointType(jh[i])
    end
    task.jh = jh
    task.jt = jt
    task.goalPose=goalPose
    task.ikTarget=simGetObjectHandle('Jaco_target')
    task.ikTip=simGetObjectHandle('Jaco_tip')
    task.ikGroup=simGetIkGroupHandle('Jaco_ik')


    task.collisionPairs={simGetCollectionHandle('Jaco'),simGetCollectionHandle('Jaco'),simGetCollectionHandle('Jaco'),sim_handle_all}
    if not collisionChecking then
        task.collisionPairs={}
    end

    --Robot arm configuration for moving to the target pose
    local goalConfigs=findSeveralCollisionFreeConfigs(task)

    if not goalConfigs then
        return {0},{},{'No configurations found'},''
    end
    path,lengths=findShortestPath(task,goalConfigs)
    if not path then
        return {0},{},{'No path found'},''

    end
    task.forbidLevel=visualizePath(task,path)
    --executeMotion(task,path,lengths)
    --local robotHandle=inInts[1]
    --local fullRobotName=simGetObjectName(robotHandle)
    --simSetStringSignal(fullRobotName..'runPath',simPackFloatTable(inFloats))


    return {0},{},{'Successful'},''
end

moveToBlock=function(inInts,inFloats,inStrings,inBuffer)
    local task={}
    task.robotHandle=inInts[1]

    local collisionChecking=inInts[2]>0
    task.maxVel=inInts[3]
    task.maxAccel=inInts[4]
    task.maxJerk=inInts[5]
    task.forbidLevel=inInts[6]
    task.metric={0.2,1,0.8,0.1,0.1,0.1}
    task.ikSteps=inInts[7]
    task.trialCnt=inInts[8]
    task.maxConfigs=inInts[9]
    task.numberOfOMPLCalculationsPasses=inInts[10]
    task.maxOMPLCalculationTime=inInts[11]



    local currentState={}
    for i=1,6,1 do currentState[i]=inFloats[i] end
    task.currentState=currentState
    local goalPose={}
    for i=1,12,1 do goalPose[i]=inFloats[i+6] end
    task.goalPose=goalPose



    jh={-1,-1,-1,-1,-1,-1}
    jt={-1,-1,-1,-1,-1,-1}
    for i=1,6,1 do
        jh[i]=simGetObjectHandle('Jaco_joint'..i)
        jt[i]=simGetJointType(jh[i])
    end
    task.jh = jh
    task.jt = jt
    task.goalPose=goalPose
    task.ikTarget=simGetObjectHandle('Jaco_target')
    task.ikTip=simGetObjectHandle('Jaco_tip')
    task.ikGroup=simGetIkGroupHandle('Jaco_ik')


    task.collisionPairs={simGetCollectionHandle('Jaco'),simGetCollectionHandle('Jaco'),simGetCollectionHandle('Jaco'),sim_handle_all}
    if not collisionChecking then
        task.collisionPairs={}
    end



    path,lengths=generateIkPath(task,true)
    executeMotion(task,path,lengths)

    return {0},{},{'Successful'},''
end

generateIkPath=function(task,ignoreCollisions)
    -- Generates (if possible) a linear, collision free path between a robot config and a target pose
    --task.forbidLevel = forbidThreadSwitches(true,task.forbidLevel)
    local currentConfig=getConfig(task.jh)
    setConfig(task.jh,task.currentState)
    simSetObjectMatrix(task.ikTarget,-1,task.goalPose)
    local coll=task.collisionPairs
    if ignoreCollisions then
        coll=nil
    end
    local c=simGenerateIkPath(task.ikGroup,task.jh,task.ikSteps,coll)
    setConfig(task.jh,currentConfig)
    -- task.forbidLevel =  forbidThreadSwitches(false,task.forbidLevel)
    if c then
        return c, generatePathLengths(task,c)
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
executeMotion=function(task,path,lengths)
    dt=simGetSimulationTimeStep()

    -- 1. Make sure we are not going too fast for each individual joint (i.e. calculate a correction factor (velCorrection)):
    jointsUpperVelocityLimits={}
    for j=1,6,1 do
        res,jointsUpperVelocityLimits[j]=simGetObjectFloatParameter(task.jh[j],sim_jointfloatparam_upper_limit)
    end
    velCorrection=1

    simSetThreadSwitchTiming(200)
    while true do
        posVelAccel={0,0,0}
        targetPosVel={lengths[#lengths],0}
        pos=0
        res=0
        previousQ={path[1],path[2],path[3],path[4],path[5],path[6]}
        local rMax=0
        rmlHandle=simRMLPos(1,0.0001,-1,posVelAccel,{task.maxVel*velCorrection,task.maxAccel,task.maxJerk},{1},targetPosVel)
        while res==0 do
            res,posVelAccel,sync=simRMLStep(rmlHandle,dt)
            if (res>=0) then
                l=posVelAccel[1]
                for i=1,#lengths-1,1 do
                    l1=lengths[i]
                    l2=lengths[i+1]
                    if (l>=l1)and(l<=l2) then
                        t=(l-l1)/(l2-l1)
                        for j=1,6,1 do
                            q=path[6*(i-1)+j]+_getJointPosDifference(path[6*(i-1)+j],path[6*i+j],task.jt[j]==sim_joint_revolute_subtype)*t
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
    simSetThreadSwitchTiming(2)

    -- 2. Execute the movement:
    posVelAccel={0,0,0}
    targetPosVel={lengths[#lengths],0}
    pos=0
    res=0
    jointPos={}
    rmlHandle=simRMLPos(1,0.0001,-1,posVelAccel,{task.maxVel*velCorrection,task.maxAccel,task.maxJerk},{1},targetPosVel)
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
                    for j=1,6,1 do
                        jointPos[j]=path[6*(i-1)+j]+_getJointPosDifference(path[6*(i-1)+j],path[6*i+j],task.jt[j]==sim_joint_revolute_subtype)*t
                    end
                    _applyJoints(task.jh,jointPos)
                    break
                end
            end
        end
        simSwitchThread()
    end
    simRMLRemove(rmlHandle)
end

findPath=function(task,goalConfigs)
    -- Here we do path planning between the specified start and goal configurations. We run the search cnt times,
    -- and return the shortest path, and its length

    -- Following because the robot has "strange" joint limits, e.g. +-10'000, and searching such a large
    -- space would be inefficient for path planning
    local jointLimitsL={}
    local jointLimitsH={}
    for i=1,#task.jh,1 do
        jointLimitsL[i]=task.currentState[i]-360*math.pi/180
        if jointLimitsL[i]<-10000 then jointLimitsL[i]=-10000 end
        jointLimitsH[i]=task.currentState[i]+360*math.pi/180
        if jointLimitsH[i]>10000 then jointLimitsH[i]=10000 end
    end
    jointLimitsL[2]=47*math.pi/180
    jointLimitsH[2]=313*math.pi/180
    jointLimitsL[3]=19*math.pi/180
    jointLimitsH[3]=341*math.pi/180

    local omplTask=simExtOMPL_createTask('omplTask')
    simExtOMPL_setAlgorithm(omplTask,sim_ompl_algorithm_BKPIECE1)
    local jSpaces={}
    for i=1,#task.jh,1 do
        local proj=i
        if i>3 then proj=0 end
        jSpaces[#jSpaces+1]=simExtOMPL_createStateSpace('j_space'..i,sim_ompl_statespacetype_joint_position,task.jh[i],{jointLimitsL[i]},{jointLimitsH[i]},proj)
    end
    simExtOMPL_setStateSpace(omplTask,jSpaces)
    simExtOMPL_setCollisionPairs(omplTask,task.collisionPairs)
    simExtOMPL_setStartState(omplTask,task.currentState)
    simExtOMPL_setGoalState(omplTask,goalConfigs[1])
    for i=2,#goalConfigs,1 do
        simExtOMPL_addGoalState(omplTask,goalConfigs[i])
    end
    local path=nil
    local l=999999999999
    --    forbidThreadSwitches(true)
    for i=1,task.numberOfOMPLCalculationsPasses,1 do
        local res,_path=simExtOMPL_compute(omplTask,task.maxOMPLCalculationTime,-1,200)
        if res>0 and _path then
            local _l=getPathLength(_path,task.jh,task.metric)
            if _l<l then
                l=_l
                path=_path
            end
        end
        if path then
            visualizePath(task,path)
        end
    end
    --    forbidThreadSwitches(false)
    simExtOMPL_destroyTask(omplTask)
    return path,l
end



findShortestPath=function(task,goalConfigs)
    -- This function will search for several paths between the specified start configuration,
    -- and several of the specified goal configurations. The shortest path will be returned
    local onePath,onePathLength=findPath(task,goalConfigs)
    return onePath,generatePathLengths(task,onePath)
end

generatePathLengths=function(task,path)
    -- Returns a table that contains a distance along the path for each path point
    local d=0
    local l=#task.jh
    local pc=#path/l
    local retLengths={0}
    for i=1,pc-1,1 do
        local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6],path[(i-1)*l+7]}
        local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6],path[i*l+7]}
        d=d+getConfigConfigDistance(config1,config2,task.metric)
        retLengths[i+1]=d
    end
    return retLengths
end

getConfig=function(jh)
    -- Returns the current robot configuration
    local config={}
    for i=1,#jh,1 do
        config[i]=simGetJointPosition(jh[i])
    end
    return config
end

setConfig=function(jh,config)
    -- Applies the specified configuration to the robot
    if config then
        for i=1,#jh,1 do
            simSetJointPosition(jh[i],config[i])
        end
    end
end

getPathLength=function(path,jh,metric)
    -- Returns the length of the path in configuration space
    local d=0
    local l=#jh
    local pc=#path/l
    for i=1,pc-1,1 do
        local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6]}
        local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6]}
        d=d+getConfigConfigDistance(config1,config2,metric)
    end
    return d
end

getConfigConfigDistance=function(config1,config2,metric)
    -- Returns the distance (in configuration space) between two configurations
    local d=0
    for i=1,#jh,1 do
        local dx=(config1[i]-config2[i])*metric[i]
        d=d+dx*dx
    end
    return math.sqrt(d)
end




getObjectPose=function(inInts,inFloats,inStrings,inBuffer)
    local objectHandle=inInts[1]
    local m=simGetObjectMatrix(objectHandle,-1)
    return {},m,{},''
end

getRobotState=function(inInts,inFloats,inStrings,inBuffer)

    local jh={-1,-1,-1,-1,-1,-1}
    local state={}
    for i=1,6,1 do
        jh[i]=simGetObjectHandle('Jaco_joint'..i)
        state[i]=simGetJointPosition(jh[i])
    end

    return {},state,{},''
end

if (sim_call_type==sim_childscriptcall_initialization) then
    local approachDirectionObstacle=simGetObjectHandle('approachDirectionObstacle')
    local p=simGetObjectSpecialProperty(approachDirectionObstacle)
    simSetObjectMatrix(approachDirectionObstacle,-1,simBuildIdentityMatrix())
    simSetObjectSpecialProperty(approachDirectionObstacle,simBoolOr32(p,sim_objectspecialproperty_collidable)-sim_objectspecialproperty_collidable)
end