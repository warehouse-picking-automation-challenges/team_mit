#!/usr/bin/env python
# scoop primitive:

# description: 
# The hand will first open completely. The hand will orient itself so that 
# the spatula finger is parallel to the ground. The hand will then move far enough into 
# the bin so that the nail of the spatula is directly above the floor of 
# the shelf bin. The hand will also position itself (while maintaining orientation) 
# such that the palm normal is pointing to the horizontal center of mass of 
# the object. The hand will then move downwards until the nail of the hand is 
# completely flush with the floor of the shelf bin. The hand will the move 
# towards the back wall of the shelf bin, until the tip of the nail is just 
# at the back of the bin. At this point, the object should be scooped. The 
# hand will then close until a predetermined amount of force is exerted on 
# the object.

# inputs:
# Pose and position of the object, object ID, shelf pose, position and force
# threshold for grasp.

# fails:
# # The vertical dimension of the object should be smaller than the maximum 
# gap distance between the two fingers.

import geometry_msgs.msg
import std_msgs
import json
import tf
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
from ik.ik import setSpeedByName
import rospy
import pdb
import numpy as np
import math
import tf.transformations as tfm
import gripper
import sensor_msgs.msg
# put shared function into ik.helper module
from ik.helper import getBinMouthAndFloor
from ik.roshelper import coordinateFrameTransform
from ik.helper import pauseFunc
from ik.helper import visualizeFunc
from ik.helper import getObjCOM
from ik.helper import openGripper
from ik.helper import closeGripper
from ik.roshelper import pubFrame
from ik.helper import get_bin_cnstr

def scoop(objPose = [1.95,0.25,1.4,0,0,0,1],
            binNum=3,
            objId = 0,
            bin_contents = None,
            robotConfig = None, 
            shelfPosition = [1.9116,-0.012498,-0.4971], 
            forceThreshold = 1, 
            isExecute = True,
            withPause = True,
            withVisualize = False):
    ## objPose: world position and orientation frame attached to com of object in quaternion form. XYZ
    ## objId: object identifier
    ## robotConfig: current time robot configuration
    ## shelfPosition: shelf position in world frame
    ## force threshold: amount fo force needed to say we have a grasp
    
    setSpeedByName(speedName = 'faster')
    
    joint_topic = '/joint_states' 
    planSuccess = True
    ## initialize listener rospy
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    
    # shelf variables
    if binNum <3:
        pretensionDelta = 0.03
    if binNum > 2 and binNum < 6:
        pretensionDelta = 0.009
    if binNum > 5 and binNum < 9:
        pretensionDelta = 0.009
    if binNum > 8:
        pretensionDelta = 0.03
    #pretensionDelta = 0.00
    lipHeight = 0.025
    
    # plan store
    plans = []
    ## get object position (world frame)
    objPosition = getObjCOM(objPose[0:3], objId)
        
    ## move gripper to object com outside bin along world y direction and
    ## move the gripper over the lip of the bin    
    # set tcp
    tcpXOffset = 0.018
    l2 = 0.47  
    tip_hand_transform = [tcpXOffset, 0, l2, 0,0,0] # to be updated when we have a hand design finalized
    # broadcast frame attached to tcp
    pubFrame(br, pose=tip_hand_transform, frame_id='tip', parent_frame_id='link_6', npub=5)
    # get position of the tcp in world frame
    pose_world = coordinateFrameTransform(tip_hand_transform[0:3], 'link_6', 'map', listener)
    tcpPos=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    tcpPosHome = tcpPos
    # set scoop orientation (rotate wrist)
    scoopOrientation = [0, 0.7071, 0, 0.7071]
    
    
    # set first target to move gripper in front of the object and adjust height to middle of bin
    distFromShelf = 0.05
    wristWidth = 0.0725 # this is actually half the wrist width
    (binMouth, binFloorHeight) = getBinMouthAndFloor(distFromShelf, binNum)
    pose_world = coordinateFrameTransform(binMouth[0:3], 'shelf', 'map', listener)
    binMouth=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    verticalOffsetLip = 0.00 # we need this so we don't damage the sucker
    wH = 0.075
    targetPosition = [binMouth[0], objPosition[1], binMouth[2]+tcpXOffset-wH+lipHeight-pretensionDelta]
    
    ## check to make sure we are inside the bin and not colliding with sides:
    minHeight, maxHeight, leftWall, rightWall = BinBBDims(binNum)
    leftWall = coordinateFrameTransform([leftWall,0,0], 'shelf', 'map', listener)
    leftWall = leftWall.pose.position.y
    rightWall = coordinateFrameTransform([rightWall,0,0], 'shelf', 'map', listener)
    rightWall = rightWall.pose.position.y
    
    interiorLipBin = [0,0.40,0] # define over the lip distance in shelf frame
    interiorLipBin = coordinateFrameTransform(interiorLipBin, 'shelf', 'map', listener)
    stepOverLip = interiorLipBin.pose.position.x
    fStroke = 0.20
    sStroke = 0.19
    binLengthSafety = 0.015
    
    if targetPosition[1] + 0.04 > leftWall:
        interiorLipBin = [0,0.36,0] # define over the lip distance in shelf frame
        interiorLipBin = coordinateFrameTransform(interiorLipBin, 'shelf', 'map', listener)
        stepOverLip = interiorLipBin.pose.position.x
        fStroke = 0.17
    
    if targetPosition[1] - 0.04 < leftWall:
        interiorLipBin = [0,0.36,0] # define over the lip distance in shelf frame
        interiorLipBin = coordinateFrameTransform(interiorLipBin, 'shelf', 'map', listener)
        stepOverLip = interiorLipBin.pose.position.x
        fStroke = 0.17 
        
    if targetPosition[1] + binLengthSafety > leftWall:
        targetPosition[1] = leftWall 
    
    if targetPosition[1] - binLengthSafety < rightWall:
        targetPosition[1] = rightWall
    
    frontOfObjectPtOutOfBin = targetPosition
    q_initial = robotConfig
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = scoopOrientation, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    plan = planner.plan()
    s = plan.success()                                    # Plan 0
    if s:
        print '[Scoop] move to COM in y successful'
        #~ print '[Scoop] tcp at:'
        #~ print(targetPosition)
        visualizeFunc(withVisualize, plan)
        plans.append(plan)
    else:
        print '[Scoop] move to COM in y fail'
        return (False, False)
        
    qf = plan.q_traj[-1]
    
    ## push spatula against base of bin (pre-tension)
    binFloorHeight = coordinateFrameTransform([0,0,binFloorHeight], 'shelf', 'map', listener)
    q_initial = qf
    percentTilt = 0.1 # 10 percent 
    scoopOrientation = [0, 0.7071*(1+percentTilt), 0, 0.7071*(1-percentTilt)]
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = scoopOrientation, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    plan = planner.plan()
    s = plan.success()                                    # Plan 1
    if s:
        print '[Scoop] reorient the hand'
        #~ print '[Scoop] tcp at:'
        #~ print(targetPosition)
        plans.append(plan)
        visualizeFunc(withVisualize, plan)
    else:
        print '[Scoop] reorient the hand fail'
        return (False, False)
    qf = plan.q_traj[-1]
    
    # set second target, go over the lip of the bin
    deltaX = np.add(-targetPosition[0],stepOverLip)
    targetPosition = np.add(targetPosition, [deltaX,0,0])
    q_initial = qf
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = scoopOrientation, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    plan = planner.plan()
    s = plan.success()                                    # Plan 2
    if s:
        print '[Scoop] move to inside the lip success'
        #~ print '[Scoop] tcp at:'
        #~ print(targetPosition)
        plans.append(plan)
        visualizeFunc(withVisualize, plan)
    else:
        print '[Scoop] move to inside the lip fail'
        return (False, False)
    qf = plan.q_traj[-1]
    
    ## perform bin length stroke to middle
    q_initial = qf
    
    targetPosition = np.add(targetPosition, [fStroke,0,-lipHeight])
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = scoopOrientation, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    plan = planner.plan()
    s = plan.success()                                    # Plan 3
    if s:
        print '[Scoop] stroke middle of bin success'
        #~ print '[Scoop] tcp at:'
        #~ print(targetPosition)
        plans.append(plan)
        visualizeFunc(withVisualize, plan)
    else:
        print '[Scoop] stroke middle of bin fail'
        return (False, False)
    qf = plan.q_traj[-1]    
    ## perform bin length stroke to end
    q_initial = qf
    targetPosition = np.add(targetPosition, [sStroke,0,0])
    scoopOrientation = [0, 0.7071+0.11/4, 0, 0.7071-0.11/4]
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = scoopOrientation, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    plan = planner.plan()
    s = plan.success()                                    # Plan 4
    if s:
        print '[Scoop] stroke middle to end of bin success'
        #~ print '[Scoop] tcp at:'
        #~ print(targetPosition)
        plans.append(plan)
        visualizeFunc(withVisualize, plan)
    else:
        print '[Scoop] stroke middle to end of bin fail'
        return (False, False)
        
    qf = plan.q_traj[-1]
    
    ## close gripper
    #~ closeGripper(forceThreshold)
    
    
    closeGripper(forceThreshold)
    execution_possible = True
    ## execute
    isNotInCollision = True
    for numOfPlan in range(0, len(plans)):
        plans[numOfPlan].visualize()
        if isExecute:
            if numOfPlan == 3:
                openGripper()
            pauseFunc(withPause)
            isNotInCollision = plans[numOfPlan].execute()
            if numOfPlan == 3:
                closeGripper(forceThreshold)
                plans[numOfPlan].executeBackward()
                openGripper()
                plans[numOfPlan].execute()
            
            if not isNotInCollision:          
                planFailNum = numOfPlan       
                print '[Scoop] collision detected'
                break                         
            if numOfPlan == 4:
                closeGripper(forceThreshold)
                rospy.sleep(0.5)
                while True:
                    APCrobotjoints = ROS_Wait_For_Msg(joint_topic, sensor_msgs.msg.JointState).getmsg() 
                    q0 = APCrobotjoints.position
                    
                    if len(q0) == 2 or len(q0) == 8:
                        q0 = q0[-2:]    # get last 2
                        break
                    
                gripper_q0=np.fabs(q0)
                drop_thick=0.000001 # finger gap =0.002m = .5 mm
                if gripper_q0[0] < drop_thick:
                    print '[Scoop] ***************'
                    print '[Scoop] Could not grasp'
                    print '[Scoop] ***************'
                    execution_possible = False
                else:
                    print '[Scoop] ***************'
                    print '[Scoop] Grasp Successful'
                    print '[Scoop] ***************'
                    execution_possible = True
                
    if not isNotInCollision:
        for numOfPlan in range(0, len(planFailNum)):
            plans[planFailNum-numOfPlan].executeBackward()
            
    ## retreat
    
    for numOfPlan in range(0, len(plans)):
        if withVisualize:
            plans[len(plans)-numOfPlan-1].visualizeBackward()
        if isExecute:
            pauseFunc(withPause)
            plans[len(plans)-numOfPlan-1].executeBackward()
            
    return True, execution_possible
    
def BinBBDims(binNum):
    lW =  0.035 # side lip width
    wW = 0.045  # wrist width
    wH = 0.075 # wrist height
    lH = 0.04  # bottom lip height
    
    bin_cnstr = get_bin_cnstr()
    bin_cnstr_offset =  [[0, -lW, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0],
                         [+lW, 0, 0, 0, 0, 0],
                         [0, -lW, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0],
                         [+lW, 0, 0, 0, 0, 0],
                         [0, -lW, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0],
                         [+lW, 0, 0, 0, 0, 0],
                         [0, -lW, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0],
                         [+lW, 0, 0, 0, 0, 0]]
    
    new_bin_cnstr = (np.array(bin_cnstr) + np.array(bin_cnstr_offset)).tolist()
             
    nrows = len(new_bin_cnstr)
    minHeight = new_bin_cnstr[binNum][4] + wH+lH
    maxHeight = new_bin_cnstr[binNum][5] - wH  
    leftWall  = new_bin_cnstr[binNum][1] - wW
    rightWall = new_bin_cnstr[binNum][0] + wW
    return( minHeight, maxHeight, leftWall, rightWall)
        
    
if __name__=='__main__':
    #scoop()
    rospy.init_node('listener', anonymous=True)
    
    objPoses = []
    objPoses.append([1.56, 0.0, 1.11,0,0,0,0])
    objPoses.append([1.56, 0.0, 0.91,0,0,0,0])
    objPoses.append([1.56, 0.0, 0.71,0,0,0,0])
    objPoses.append([1.56, 0.0, 0.51,0,0,0,0])
    
    binNums = [0,3,6,9]
    
    for x in range(0,4):
        scoop(objPose = objPoses[x],
            binNum = binNums[x], 
            robotConfig=None, 
            isExecute = True,
            withPause = True)

# objPose =  [1.60593056679, 0.29076179862, 0.863177359104], binNum = 3
# objPose =  [1.55620419979, 0.281148612499, 1.14214038849], binNum = 0, 
# obJPose =  [1.62570548058, 0.289612442255, 0.648919522762], binNum = 6,
