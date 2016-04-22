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
import rospy
import pdb
import numpy as np
import math
import tf.transformations as tfm

from ik.helper import getBinMouthAndFloor
from ik.roshelper import coordinateFrameTransform
from ik.helper import pauseFunc
from ik.helper import visualizeFunc
from ik.helper import getObjCOM
from ik.helper import openGripper
from ik.helper import closeGripper

def scoop(objPose = [1.95,0.25,1.4], objId = 0, binNum=3, robotConfig=[0,0,0,0,0,0], shelfPosition = [1.91,0,-0.50], forceThreshold = 1, isExecute = False):
    ## objPose: world position and orientation frame attached to com of object in quaternion form. XYZ. This is a list, not a matrix
    ## objId: object identifier
    ## robotConfig: current time robot configuration. List of joint angles, in radians
    ## shelfPosition: shelf position in world frame. This is a list.
    ## force threshold: amount of force needed to say we have a grasp

    ## initialize listener rospy
    ## Copy pasta hasta la pasta
    rospy.init_node('listener', anonymous=True)
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    ##hasta la copy la patio
    ## stop pasta
    
    # shelf variables
    pretensionDelta = 0.00 #how far we push nail down
    lipHeight = 0.035 #height of the lip of the bin
    
    # plan store
    plans = [] #list of plans generated each plan is an object
    ## get object position (world frame)
    objPosition = getObjCOM(objPose, objId) #
        
    ## move gripper to object com outside bin along world y direction and
    ## move the gripper over the lip of the bin    
    # set tcp. this is the XYZ roll-pitch-yaw of middle of the wrist with respect to link 6
    l2 = 0.45  #how far tcp is away from link 6
    tip_hand_transform = [0, 0, l2, 0,0,0] #something used for planning in drake, I don't get it
    # to be updated when we have a hand design finalized
    # broadcast frame attached to tcp
    rospy.sleep(0.1) #.1 second delay so broadcasting works. syncing with ROS
    #broadcasting transformation between tcp and link 6 on ROS
    br.sendTransform(tuple(tip_hand_transform[0:3]), tfm.quaternion_from_euler(*tip_hand_transform[3:6]), rospy.Time.now(), 'tip', "link_6")
    rospy.sleep(0.1)
    # get position of the tcp in world frame
    pose_world = coordinateFrameTransform(tip_hand_transform[0:3], 'link_6', 'map', listener) 
    #tcp position as a list
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
    targetPosition = [binMouth[0], objPosition[1], binMouth[2]] #align the position of the tcp relative to the object before the scoop
    frontOfObjectPtOutOfBin = targetPosition
    q_initial = robotConfig
    #q0 is initial configuration
    #target_tip_pos is where we want the tcp to be
    #target_tip_ori is the tip orientation
    #tip hand transform is see above
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = scoopOrientation, tip_hand_transform=tip_hand_transform)
    plan = planner.plan()
    s = plan.success() #true if successful
    if s:
        print 'move to COM in y successful'
        print 'tcp at:'
        print(targetPosition)
        plan.visualize() #display in arvix
        plans.append(plan)
        if isExecute:
            plan.execute() #this executes the plan for reals
    else:
        print 'move to COM in y fail'
        #after this point, it's just a sequence of points for the plan
    qf = plan.q_traj[-1] #this is the final configuration of the object
    # set second target, go over the lip of the bin
    interiorLipBin = [0,0.37,0] # define over the lip distance in shelf frame
    interiorLipBin = coordinateFrameTransform(interiorLipBin, 'shelf', 'map', listener)
    deltaX = np.add(-targetPosition[0],interiorLipBin.pose.position.x)
    targetPosition = np.add(targetPosition, [deltaX,0,0])
    q_initial = qf
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = scoopOrientation, tip_hand_transform=tip_hand_transform)
    plan = planner.plan()
    s = plan.success()
    if s:
        print 'move to inside the lip success'
        print 'tcp at:'
        print(targetPosition)
        plans.append(plan)
        plan.visualize()
        if isExecute:
            plan.execute()
    else:
        print 'move to inside the lip fail'
    qf = plan.q_traj[-1]
    
    ## open gripper fully
    openGripper()
    
    ## push spatula against base of bin (pre-tension)
    binFloorHeight = coordinateFrameTransform([0,0,binFloorHeight], 'shelf', 'map', listener)
    q_initial = qf
    deltaH = targetPosition[2] - binFloorHeight.pose.position.z - pretensionDelta - lipHeight
    targetPosition = np.add(targetPosition, [0,0,-deltaH+wristWidth])
    scoopOrientation = [0, 0.7071+0.11/2, 0, 0.7071-0.11/2]
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = scoopOrientation, tip_hand_transform=tip_hand_transform)
    plan = planner.plan()
    s = plan.success()
    if s:
        print 'push finger against floor of bin success'
        print 'tcp at:'
        print(targetPosition)
        plans.append(plan)
        plan.visualize()
        if isExecute:
            plan.execute()
    else:
        print 'push finger against floor of bin fail'
    qf = plan.q_traj[-1]
    
    ## perform bin length stroke to middle
    q_initial = qf
    targetPosition = np.add(targetPosition, [0.20,0,-lipHeight])
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = scoopOrientation, tip_hand_transform=tip_hand_transform)
    plan = planner.plan()
    s = plan.success()
    if s:
        print 'stroke middle of bin success'
        print 'tcp at:'
        print(targetPosition)
        plans.append(plan)
        plan.visualize()
        if isExecute:
            plan.execute()
    else:
        print 'stroke middle of bin fail'
    qf = plan.q_traj[-1]
    plans.append(plan)
    
    ## perform bin length stroke to end
    q_initial = qf
    targetPosition = np.add(targetPosition, [0.20,0,0])
    scoopOrientation = [0, 0.7071+0.11/4, 0, 0.7071-0.11/4]
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = scoopOrientation, tip_hand_transform=tip_hand_transform)
    plan = planner.plan()
    s = plan.success()
    if s:
        print 'stroke middle to end of bin success'
        print 'tcp at:'
        print(targetPosition)
        plans.append(plan)
        plan.visualize()
        if isExecute:
            plan.execute()
    else:
        print 'stroke middle to end of bin fail'
    qf = plan.q_traj[-1]
    plans.append(plan)
    
    ## close gripper
    closeGripper(forceThreshold)
    
    ## retreat
    #pdb.set_trace() #TRACEEEEE
    #this does each plan backwards so that the hand can leave the bin
    #VERY IMPORTANT
    for numOfPlan in range(0, len(plans)):
        plans[len(plans)-numOfPlan-1].visualizeBackward()
    if isExecute:
        for numOfPlan in range(0, len(plans)):
            plans[len(plans)-numOfPlan-1].executeBackward()
    


    
if __name__=='__main__':
    scoop()

