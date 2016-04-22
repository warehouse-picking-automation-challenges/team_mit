#!/usr/bin/env python
# topple primitive:

# description: 
# The hand is rotated such that the normals of its large faces are
# vertical (note this could actually be two configurations).
# The hand will position itself in fron of the target object and above
# its COM while staying outside the bin. With the gripper closed, the
# hand will move into the bin and push the target object to topple it.
# Keeping the gripper closed, the hand will be retracted from the bin.

# inputs:
# Pose and position of the object, object ID, shelf pose, position and force
# threshold for grasp.

# fails:
# The vertical dimension of the object should be smaller than the maximum 
# gap distance between the two fingers.

import geometry_msgs.msg
import std_msgs
import json
import tf
from ik.ik import setSpeedByName
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
import rospy
import pdb
import numpy as np
import math
import tf.transformations as tfm
import gripper

# put shared function into ik.helper module
from ik.helper import getBinMouthAndFloor
from ik.helper import getMaxHeight
from ik.helper import getMinHeight
from ik.roshelper import coordinateFrameTransform
from ik.helper import pauseFunc
from ik.helper import visualizeFunc
from ik.helper import getObjCOM
from ik.helper import openGripper
from ik.helper import closeGripper
from ik.helper import get_bin_cnstr
from ik.roshelper import pubFrame

def topple(objPose = [1.55,0.25,1.1,0,0,0,0],
            binNum=0,
            objId = 0, 
            bin_contents = None,
            robotConfig = None,
            shelfPosition = [1.9116,-0.012498,-0.4971],
            forceThreshold = 1,
            binDepthReach = 0.40,
            isExecute = True,
            withPause = True,
            withVisualize = True):
    ## objPose: world position and orientation frame attached to com of object
    ## objId: object identifier
    ## robotConfig: current time robot configuration
    ## shelfPosition: shelf position in world frame
    ## force threshold: amount fo force needed to say we have a grasp
    planSuccess = True
    ## initialize listener rospy
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    
    setSpeedByName(speedName = 'faster')
    joint_topic = '/joint_states'
        
    # shelf variables
    pretensionDelta = 0.00
    lipHeight = 0.035
    
    temp_bias = 0.00 # TO-DO remove this:
    
    # plan store
    plans = []
    
    ## get object position (world frame)
    objPosition = getObjCOM(objPose[0:3], objId)
        
    ## move gripper to object com outside bin along world y direction and
    ## move the gripper over the lip of the bin    
    # set tcp
    l1 = 0.018
    l2 = 0.470  
    tip_hand_transform = [l1, 0, l2, 0,0,0] # to be updated when we have a hand design finalized
    
    # broadcast frame attached to tcp
    pubFrame(br, pose=tip_hand_transform, frame_id='tip', parent_frame_id='link_6', npub=5)
    
    # get position of the tcp in world frame
    pose_world = coordinateFrameTransform(tip_hand_transform[0:3], 'link_6', 'map', listener)
    tcpPos=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    tcpPosHome = tcpPos
    toppleOrientation = [0.0, 0.7071, 0.0, 0.7071] # set topple orientation (rotate wrist)
    closeGripper(forceThreshold) # if gripper open close it
    
    # set first target to move gripper in front of the object and adjust height to middle of bin
    distFromShelf   = 0.05
    wristWidth      = 0.0725 # this is actually half the wrist width
    (binMouth, binFloorHeight)  = getBinMouthAndFloor(distFromShelf, binNum)
    pose_world                  = coordinateFrameTransform(binMouth[0:3], 'shelf', 'map', listener)
    binFloorHeight              = coordinateFrameTransform([0,0,binFloorHeight], 'shelf', 'map', listener)
    binFloorHeight              = binFloorHeight.pose.position.z
    binMouth                    = [pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    
    # set offset past COM
    # offset is 1/4 of object's height + half finger width
    # note: (COM height - bin floor height) is half of the object's height
    fingerWidth     = 0.06
    offsetFinger    = fingerWidth/2
    targetHeight    = (objPosition[2] - binFloorHeight)*2 # to be changed to be from an object
    
    offsetEffect = targetHeight*3/4

    offset = offsetEffect
    
    vertToppleHeight = binFloorHeight+offset-temp_bias
    
    ## bounding box constraint
    minHeight, maxHeight, leftWall, rightWall = BinBBDims(binNum) # shelf frame
    # trying to topple an object that is too tall?
    maxHeight = coordinateFrameTransform([0,0,maxHeight], 'shelf', 'map', listener)
    
    #pdb.set_trace()
    
    targetPosition = [binMouth[0], objPosition[1], vertToppleHeight]
    binDepthReach = 0.3 
    
    binHeightSaftey = 0.01
    if vertToppleHeight + binHeightSaftey > maxHeight.pose.position.z:
        fingerLength = 0.26
        binDepth = 0.42
        binDepthReach = binDepth - fingerLength + 0.08
        toppleHeightWhenTall = binMouth[2] + 0.03
        targetPosition = [binMouth[0], objPosition[1], toppleHeightWhenTall]
        print '[Topple] Object too tall, topple will not go all the way in'
    
    # trying to topple an object that is too short?
    minHeight = coordinateFrameTransform([0,0,minHeight], 'shelf', 'map', listener)
    
    closeGripper(forceThreshold)
    
    if vertToppleHeight - binHeightSaftey < minHeight.pose.position.z:
        print '[Topple] Object too short, resetting topple height'
        toppleHeightWhenShort = minHeight.pose.position.z + binHeightSaftey 
        targetPosition = [binMouth[0], objPosition[1], toppleHeightWhenShort]
        gripper.move(80,100)
    
    # too far left or right?
    leftWall = coordinateFrameTransform([leftWall,0,0], 'shelf', 'map', listener)
    leftWall = leftWall.pose.position.y
    rightWall = coordinateFrameTransform([rightWall,0,0], 'shelf', 'map', listener)
    rightWall = rightWall.pose.position.y
    
    binLengthSafety = 0.02
    horizontalSaftey = 0.01
    if targetPosition[1] + binLengthSafety > leftWall:
        targetPosition[1] = leftWall - horizontalSaftey
    
    if targetPosition[1] - binLengthSafety < rightWall:
        targetPosition[1] = rightWall + horizontalSaftey

    frontOfObjectPtOutOfBin = targetPosition
    q_initial = robotConfig
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = toppleOrientation, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    plan = planner.plan()
    s = plan.success()
    
    effect = 'topple'
    if s and effect == 'topple':
        print '[Topple] move to COM in y and above COM in z successful'
    elif s and effect == 'slide':
        print '[Topple] move to COM in y and below COM in z successful'
    else:
        print '[Topple] move to COM in y and below COM in z successful' #Default is slide, made three cases for flexibility
    
    if s:
        visualizeFunc(withVisualize, plan)
        plans.append(plan)
    else:
        print '[Topple] move to COM in y and above COM in z fail'
        planSuccess = False
        
    qf = plan.q_traj[-1]
        
    ## perform bin length stroke to end
    q_initial = qf
    targetPosition = np.add(targetPosition, [binDepthReach,0,0])
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = toppleOrientation, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    plan = planner.plan()
    s = plan.success()
    if s:
        print '[Topple] stroke to end of bin success'
        visualizeFunc(withVisualize, plan)
        plans.append(plan)
    else:
        print '[Topple] stroke to end of bin fail'
        planSuccess = False
    qf = plan.q_traj[-1]
    
    ## close gripper
    closeGripper(forceThreshold)
    
    ## retreat
    # go back along stroke
    q_initial = qf
    targetPosition = frontOfObjectPtOutOfBin
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = toppleOrientation, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    plan = planner.plan()
    s = plan.success()
    if s:
        print '[Topple] stroke to bin start success'
        visualizeFunc(withVisualize, plan)
        plans.append(plan)
    else:
        print '[Topple] stroke to bin start fail'
        planSuccess = False
    qf = plan.q_traj[-1]
    
    # go back to initial tcp position
    q_initial = qf
    targetPosition = frontOfObjectPtOutOfBin
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = toppleOrientation, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    plan = planner.plan()
    s = plan.success()
    if s:
        print '[Topple] return to mouth success'
        visualizeFunc(withVisualize, plan)
        plans.append(plan)
    else:
        print '[Topple] return to mouth fail'
        planSuccess = False
    
    if not planSuccess:
        return False, False
    
    ## execute
    #~ for numOfPlan in range(0, len(plans)):
        #~ plans[numOfPlan].visualize()
        #~ if isExecute:
            #~ pauseFunc(withPause)
            #~ plans[numOfPlan].execute()
    
    ## execute
    isNotInCollision = True
    for numOfPlan in range(0, len(plans)):
        plans[numOfPlan].visualize()
        if isExecute:
            pauseFunc(withPause)
            isNotInCollision = plans[numOfPlan].execute()
            if not isNotInCollision:
                print '[Topple] collision detected'
                planFailNum = numOfPlan
                break
                
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
            
    return True, False


def BinBBDims(binNum):
    lW =  0.035 
    wW = 0.03
    wH = 0.075
    lH = 0.04
    
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
    rospy.init_node('listener', anonymous=True)
    
    objPoses = []
    objPoses.append([1.66, 0.0, 1.51,0,0,0,0])
    objPoses.append([1.66, 0.0, 0.91,0,0,0,0])
    objPoses.append([1.66, 0.0, 0.71,0,0,0,0])
    objPoses.append([1.66, 0.0, 0.51,0,0,0,0])
    
    binNums = [0,3,6,9]
    
    for x in range(0,4):
        topple(objPose =  objPoses[x],
                objId = 02, 
                binNum = binNums[x], 
                robotConfig=None,
                forceThreshold = 1,
                binDepthReach = 0.37,
                isExecute = True,
                withPause = False)
