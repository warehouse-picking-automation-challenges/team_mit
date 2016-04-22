#!/usr/bin/env python
# helper script for manual calibration
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
from ik.ik import Plan
from ik.helper import getBinMouthAndFloor
from ik.roshelper import coordinateFrameTransform

def manual_cal_1(binNum = 0, strPos = None):
    
    if strPos  == None:
        print 'No desired position entered'
        return
        
    ## initialize listener rospy
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    joint_topic = '/joint_states'
    
    # set tcp
    l1 = 0.06345
    l2 = 0.400  
    tip_hand_transform = [l1, 0, l2, 0,0,0] # to be updated when we have a hand design finalized
    
    # broadcast frame attached to tcp
    rospy.sleep(0.1)
    br.sendTransform(tuple(tip_hand_transform[0:3]), tfm.quaternion_from_euler(*tip_hand_transform[3:6]), rospy.Time.now(), 'tip', "link_6")
    rospy.sleep(0.1)
    
    # get position of the tcp in world frame
    pose_world  = coordinateFrameTransform(tip_hand_transform[0:3], 'link_6', 'map', listener)
    tcpPos      =[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    tcpPosHome  = tcpPos
    orientation = [0.0, 0.7071, 0.0, 0.7071] # set topple orientation (rotate wrist)
    
    rospy.sleep(0.1)
    br.sendTransform(tuple(tip_hand_transform[0:3]), tfm.quaternion_from_euler(*tip_hand_transform[3:6]), rospy.Time.now(), 'tip', "link_6")
    rospy.sleep(0.1)
    
    pose_world = coordinateFrameTransform(tip_hand_transform[0:3], 'link_6', 'map', listener)
    tcpPos=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    
    print 'TCP is at:', tcpPos
    
    distFromShelf = 0.05
    mouthBinShelf, binBaseHeightShelf = getBinMouthAndFloor(distFromShelf, binNum)
    mouthBin = coordinateFrameTransform(mouthBinShelf, 'shelf', 'map', listener)
    mouthBin = [mouthBin.pose.position.x, mouthBin.pose.position.y, mouthBin.pose.position.z]
    
    # define modes here:
    
    # go home
    if strPos  == 'home':
        print 'planning to go home'
        plan = Plan()
        plan.q_traj = [[0, -0.3959, 0.58466, 0.03,   0.1152, -0.1745]]
        plan.visualize()
        raw_input('execute?')
        plan.execute()
        print 'done going home'
    
    # go mouth
    if strPos  == 'mouth':
        print 'planning to go to mouth'
        Orientation = [0.0, 0.7071, 0.0, 0.7071]
        targetPt = mouthBin
        planner = IK(q0 = None, target_tip_pos = targetPt, 
                     target_tip_ori = Orientation, tip_hand_transform=tip_hand_transform, 
                     joint_topic=joint_topic)
        plan = planner.plan()
        plan.visualize()
        s = plan.success()
        if s:
            raw_input('execute?')
            plan.execute()
        if not s:
            print 'could not find plan'
        print 'done going to mouth'
    
    # go mid back bin
    if strPos  == 'shelf':
        print 'planning to go to mid back of the bin'
        Orientation = [0.0, 0.7071, 0.0, 0.7071]
        targetPt = [mouthBin[0], mouthBin[1], mouthBin[2]] 
        planner = IK(q0 = None, target_tip_pos = targetPt, 
                     target_tip_ori = Orientation, tip_hand_transform=tip_hand_transform, 
                     joint_topic=joint_topic)
        plan = planner.plan()
        plan.visualize()
        s = plan.success()
        if s:
            raw_input('execute?')
            plan.execute()
        if not s:
            print 'could not find plan'
        print 'done going to mid back of the bin'


if __name__=='__main__':
    rospy.init_node('listener', anonymous=True)
    
    # available options:
    # mouth, home, shelf
    manual_cal_1(binNum = 2, strPos = 'home')
