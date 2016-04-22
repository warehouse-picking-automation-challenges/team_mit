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
from ik.roshelper import coordinateFrameTransform
from ik.helper import pauseFunc
from ik.helper import visualizeFunc
from ik.helper import getObjCOM
from ik.helper import openGripper
from ik.helper import closeGripper
from ik.roshelper import pubFrame

def push(objPose = [1.55,0.25,1.1,0,0,0,0],
            binNum=0,
            objId = 0, 
            bin_contents = None, 
            robotConfig=None,
            shelfPosition = [1.9116,-0.012498,-0.4971],
            forceThreshold = 1,
            binDepthReach = 0.40,
            isExecute = True,
            withPause = False):
    ## objPose: world position and orientation frame attached to com of object
    ## objId: object identifier
    ## robotConfig: current time robot configuration
    ## shelfPosition: shelf position in world frame
    ## force threshold: amount fo force needed to say we have a grasp

    ## initialize listener rospy
    rospy.init_node('listener', anonymous=True)
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    
    joint_topic = '/joint_states'
        
    # shelf variables
    pretensionDelta = 0.00
    lipHeight = 0.035
    ## get object position (world frame)
    objPosition = getObjCOM(objPose[0:3], objId)
        
    ## move gripper to object com outside bin along world y direction and
    ## move the gripper over the lip of the bin    
    # set tcp
    l2 = 0.47  
    tip_hand_transform = [0, 0, l2, 0,0,0] # to be updated when we have a hand design finalized
    # broadcast frame attached to tcp
    pubFrame(br, pose=tip_hand_transform, frame_id='tip', parent_frame_id='link_6', npub=5)
    # get position of the tcp in world frame
    pose_world = coordinateFrameTransform(tip_hand_transform[0:3], 'link_6', 'map', listener)
    tcpPos=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    tcpPosHome = tcpPos
    # set topple orientation (rotate wrist)
    toppleOrientation = [0.0, 0.7071, 0.0, 0.7071] #[0, 0.7071, 0, 0.7071]
    
    # if gripper open close it
    closeGripper(forceThreshold)
    
    # set first target to move gripper in front of the object and adjust height to middle of bin
    distFromShelf = 0.05
    wristWidth = 0.0725 # this is actually half the wrist width
    (binMouth, binFloorHeight) = getBinMouthAndFloor(distFromShelf, binNum)
    pose_world = coordinateFrameTransform(binMouth[0:3], 'shelf', 'map', listener)
    binFloorHeight = coordinateFrameTransform([0,0,binFloorHeight], 'shelf', 'map', listener)
    binFloorHeight = binFloorHeight.pose.position.z
    binMouth=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    
    # set offset past COM
    # offset is 1/4 of object's height + half finger width
    # note: (COM height - bin floor height) is half of the object's height
    #pdb.set_trace() #TRACEEEEE
    fingerWidth = 0.06
    offsetFinger = fingerWidth/2
    targetHeight = (objPosition[2] - binFloorHeight)*2
    effect = 'slide'
    if effect == 'topple':
        offsetEffect = targetHeight*3/4
    elif effect == 'slide':
        offsetEffect = targetHeight*1/4
    else:
        offsetEffect = targetHeight*1/4 #Default is slide, made three cases for flexibility
    
    offset = offsetFinger + offsetEffect
    targetPosition = [binMouth[0], objPosition[1], binFloorHeight+offset]
    frontOfObjectPtOutOfBin = targetPosition
    q_initial = robotConfig
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = toppleOrientation, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    plan = planner.plan()
    s = plan.success()
    
    if s and effect == 'topple':
        print 'move to COM in y and above COM in z successful'
    elif s and effect == 'slide':
        print 'move to COM in y and below COM in z successful'
    else:
        print 'move to COM in y and below COM in z successful' #Default is slide, made three cases for flexibility
    
    if s:
        print 'tcp at:'
        print(targetPosition)
        plan.visualize()
        if isExecute:
            pauseFunc(withPause)
            plan.execute()
    else:
        print 'move to COM in y and above COM in z fail'
        
    qf = plan.q_traj[-1]
    # set second target, go over the lip of the bin
    #interiorLipBin = [0,0.39,0] # define over the lip distance in shelf frame
    #interiorLipBin = coordinateFrameTransform(interiorLipBin, 'shelf', 'map', listener)
    #deltaX = np.add(-targetPosition[0],interiorLipBin.pose.position.x)
    #targetPosition = np.add(targetPosition, [deltaX,0,0])
    #q_initial = qf
    #planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = toppleOrientation, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    #plan = planner.plan()
    #s = plan.success()
    #if s:
    #    print 'move to inside the lip success'
    #    print 'tcp at:'
    #    print(targetPosition)
    #    plan.visualize()
    #    if isExecute:
    #        pauseFunc(withPause)
    #        plan.execute()
    #else:
    #    print 'move to inside the lip fail'
    #qf = plan.q_traj[-1]
    
    ## close gripper fully (Do we wish to topple with gripper slightly open?)
    closeGripper(forceThreshold)
    
    ## perform bin length stroke to end
    q_initial = qf
    targetPosition = np.add(targetPosition, [binDepthReach,0,0])
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = toppleOrientation, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    plan = planner.plan()
    s = plan.success()
    if s:
        print 'stroke to end of bin success'
        print 'tcp at:'
        print(targetPosition)
        plan.visualize()
        if isExecute:
            pauseFunc(withPause)
            plan.execute()
    else:
        print 'stroke to end of bin fail'
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
        print 'stroke to bin start success'
        print 'tcp at:'
        print(targetPosition)
        plan.visualize()
        if isExecute:
            pauseFunc(withPause)
            plan.execute()
    else:
        print 'stroke to bin start fail'
    qf = plan.q_traj[-1]
    
    # go back to initial tcp position
    q_initial = qf
    targetPosition = frontOfObjectPtOutOfBin
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = toppleOrientation, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    plan = planner.plan()
    s = plan.success()
    if s:
        print 'return to mouth success'
        plan.visualize()
        if isExecute:
            pauseFunc(withPause)
            plan.execute()
    else:
        print 'return to mouth fail'
    
    return False
    
    
if __name__=='__main__':
    temp_bias = 0.04  #TO-DO: Remove this bias, calculate object height correctly
    push(objPose = [1.55620419979, 0.281148612499, 1.14214038849-temp_bias,0,0,0,0],
            objId = 0, 
            binNum=0, 
            robotConfig=None,
            forceThreshold = 1,
            binDepthReach = 0.37,
            isExecute = True,
            withPause = False)
