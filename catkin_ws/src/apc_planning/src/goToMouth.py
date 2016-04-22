#!/usr/bin/env python
# goToMouth primitive:

# description: 
# called when we want to go to mouth of bin, BE VERY CAREFUL IF CALLED INCORRECTLY
# OR AT A BAD TIME IT MAY HIT THE CAMERAS OR THE SHELF, designed to be used
# from home or from the objective bin, not when hand is inside the bin

# the robot moves in essentially a straight line from where it is to the home
# tcp pose (position is taken as argument, pose is assumed to be gripper open
# toward shelf)

# inputs:
# configuration of the robot when called, assumed to be at the mouth bin or
# at objective bin
# location of the home, xyz in world coordinate frame and orientation, this
# is subject to change

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

def goToMouth(robotConfig = None,
             binNum = 0, 
             isExecute = True,
             withPause = False):
    ## robotConfig: current time robot configuration
    joint_topic = '/joint_states'
    
    ## initialize listener rospy
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
            
    # plan store
    plans = []
       
    ## initial variable and tcp definitions
    # set tcp
    l2 = 0.47  
    tip_hand_transform = [0, 0, l2, 0,0,0] # to be updated when we have a hand design finalized
    # broadcast frame attached to tcp
    pubFrame(br, pose=tip_hand_transform, frame_id='tip', parent_frame_id='link_6', npub=5)
    # get position of the tcp in world frame
    pose_world = coordinateFrameTransform(tip_hand_transform[0:3], 'link_6', 'map', listener)
    tcpPos=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    tcpPosHome = tcpPos
    # set home orientation
    gripperOri = [0, 0.7071, 0, 0.7071]
    
    # move to bin mouth
    distFromShelf = 0.15
    mouthPt,temp = getBinMouthAndFloor(distFromShelf, binNum)
    mouthPt = coordinateFrameTransform(mouthPt, 'shelf', 'map', listener)
    targetPosition = [mouthPt.pose.position.x, mouthPt.pose.position.y, mouthPt.pose.position.z]
    q_initial = robotConfig
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = gripperOri, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    plan = planner.plan()
    s = plan.success()
    if s:
        print '[goToMouth] move to bin mouth successful'
        plan.visualize()
        plans.append(plan)
        if isExecute:
            pauseFunc(withPause)
            plan.execute()
    else:
        print '[goToMouth] move to bin mouth fail'
        return None
        
    qf = plan.q_traj[-1]
    
    ## open gripper fully
    openGripper()
    
    return plan
    
    


if __name__=='__main__':
    rospy.init_node('listener', anonymous=True)
    goToMouth(robotConfig=None,
            binNum = 0, 
            isExecute = True,
            withPause = False)

# objPose =  [1.60593056679, 0.29076179862, 0.863177359104], binNum = 3
# objPose =  [1.55620419979, 0.281148612499, 1.14214038849], binNum = 0, 
# obJPose =  [1.62570548058, 0.289612442255, 0.648919522762], binNum = 6,
