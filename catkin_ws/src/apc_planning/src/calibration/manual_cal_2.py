#!/usr/bin/env python
# helper script for manual calibration
import geometry_msgs.msg
import std_msgs
import json
import tf
import sys
sys.path.append("../") ; import goToHome
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
from ik.ik import IKJoint
from ik.ik import Plan
import rospy
import pdb
import numpy as np
import math
import tf.transformations as tfm
import gripper
from ik.ik import Plan
from ik.helper import getBinMouthAndFloor
from ik.roshelper import coordinateFrameTransform
import ik.ik
import os


def manual_cal_2(binNum = 0, endBinNum = 11):
    
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
    
    moveDistHorizontal = 0.09 #we could do 1 more cm (without the lip we could move about 4cm more)
    moveDistVertical = 0.08 #we could do 1 more cm
    
    #set Speed
    ik.ik.setSpeedByName('faster')
    
    # broadcast frame attached to tcp
    #~ rospy.sleep(0.1)
    #~ br.sendTransform(tuple(tip_hand_transform[0:3]), tfm.quaternion_from_euler(*tip_hand_transform[3:6]), rospy.Time.now(), 'calib', "link_6")
    #~ rospy.sleep(0.1)
    
    # get position of the tcp in world frame
    pose_world  = coordinateFrameTransform(tip_hand_transform[0:3], 'link_6', 'map', listener)
    tcpPos      =[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    tcpPosHome  = tcpPos
    orientation = [0.0, 0.7071, 0.0, 0.7071] # set topple orientation (rotate wrist)
    
    #~ rospy.sleep(0.1)
    #~ br.sendTransform(tuple(tip_hand_transform[0:3]), tfm.quaternion_from_euler(*tip_hand_transform[3:6]), rospy.Time.now(), 'calib', "link_6")
    #~ rospy.sleep(0.1)
    
    pose_world = coordinateFrameTransform(tip_hand_transform[0:3], 'link_6', 'map', listener)
    tcpPos=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    
    print 'TCP is at:', tcpPos
    
    print 'planning to go home'
    
    # home_joint_pose = [-0.005744439031, -0.6879946105, 0.5861570764, 0.09693312715, 0.1061231983, -0.1045031463]
    # planner = IKJoint(target_joint_pos=home_joint_pose)
    # plan = planner.plan()
    # plan.visualize()
    raw_input('execute?')
    # plan.execute()
    goToHome.goToHome()
    print 'done going home'
    
    calibBins = []
    for x in range(binNum, endBinNum+1):
        calibPts = []
        
        if x == 1 or x == 4 or x == 7 or x == 10:
            calibBins.append([])
            continue
        
        distFromShelf = 0.05
        mouthBinShelf, binBaseHeightShelf = getBinMouthAndFloor(distFromShelf, x)
        mouthBin = coordinateFrameTransform(mouthBinShelf, 'shelf', 'map', listener)
        mouthBin = [mouthBin.pose.position.x, mouthBin.pose.position.y, mouthBin.pose.position.z]
        
        epsilon = 0.016
        distFromShelf = distFromShelf - epsilon
               
        # go to mouth of bin
        print 'planning to go to mouth but a little out'
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
            ik.ik.setSpeedByName('fast')
            plan.execute()
        if not s:
            print 'could not find plan'
        print 'done going to mouth'
        
        # go to the mouth of the bin but actually inside
        print 'planning to go to start position'
        Orientation = [0.0, 0.7071, 0.0, 0.7071]
        targetPt = [mouthBin[0]+distFromShelf, mouthBin[1], mouthBin[2]] 
        planner = IK(q0 = None, target_tip_pos = targetPt, 
                     target_tip_ori = Orientation, tip_hand_transform=tip_hand_transform, 
                     joint_topic=joint_topic)
        plan = planner.plan()
        s = plan.success()
        if s:
            plan.visualize()
            raw_input('execute?')
            ik.ik.setSpeedByName('fast')
            plan.execute()
        if not s:
            print 'could not find plan'
        print 'done going to start position'
        
        
        
        OrientationList = []
        targetPtList = []
        captionList = []
        ntouch = 5
        
        # go down
        captionList.append('go down')
        Orientation = [0.0, 0.7071, 0.0, 0.7071]
        targetPt = [mouthBin[0]+distFromShelf, mouthBin[1], mouthBin[2]-moveDistVertical] 
        OrientationList.append(Orientation)
        targetPtList.append(targetPt)
        
        # go left
        captionList.append('go left')
        Orientation = [0.5, 0.5, 0.5, 0.5]
        targetPt = [mouthBin[0]+distFromShelf, mouthBin[1]+moveDistHorizontal, mouthBin[2]] 
        OrientationList.append(Orientation)
        targetPtList.append(targetPt)
        
        # go right
        captionList.append('go right')
        Orientation = [0.5, -0.5, 0.5, -0.5]
        targetPt = [mouthBin[0]+distFromShelf, mouthBin[1]-moveDistHorizontal, mouthBin[2]] 
        OrientationList.append(Orientation)
        targetPtList.append(targetPt)
        
        # go up
        captionList.append('go up')
        Orientation = [0.7071, 0, 0.7071, 0]
        targetPt = [mouthBin[0]+distFromShelf, mouthBin[1], mouthBin[2]+moveDistVertical] 
        OrientationList.append(Orientation)
        targetPtList.append(targetPt)
        
        # go to mid back of the bin
        captionList.append('go mid back')
        Orientation = [0, 0.7071+0.11/4, 0, 0.7071-0.11/4]
        targetPt = [mouthBin[0]+0.30, mouthBin[1], mouthBin[2]-0.08] 
        OrientationList.append(Orientation)
        targetPtList.append(targetPt)
        
        for touchInd in range(ntouch):
            print captionList[touchInd]
            targetPt = targetPtList[touchInd]
            Orientation = OrientationList[touchInd]
            
            planner = IK(q0 = None, target_tip_pos = targetPt, 
                         target_tip_ori = Orientation, tip_hand_transform=tip_hand_transform, 
                         joint_topic=joint_topic)
            plan = planner.plan()
            s = plan.success()
            if s:
                plan.visualize()
                raw_input('execute?')
                ik.ik.setSpeedByName('fast')
                plan.execute()
            if not s:
                print 'could not find plan'
            print 'done going down, TELEOP TIME:'
            raw_input('when done with teleop hit enter: ')
            #~ rospy.sleep(0.1)
            #~ br.sendTransform(tuple(tip_hand_transform[0:3]), tfm.quaternion_from_euler(*tip_hand_transform[3:6]), rospy.Time.now(), 'calib', "link_6")
            #~ rospy.sleep(0.1)
            t = listener.getLatestCommonTime('calib', 'map')
            (calibPoseTrans, calibPoseRot) = listener.lookupTransform('map','calib',t)
            calibPose = list(calibPoseTrans) + list(calibPoseRot)
            #~ calibPose = coordinateFrameTransform(tip_hand_transform[0:3], 'link_6', 'map', listener)
            #~ calibPtsTemp = [calibPose.pose.position.x,calibPose.pose.position.y,calibPose.pose.position.z]
            calibPts.append(calibPose)
            raw_input('done recording position, hit enter to go back')
            
            if s:
                plan.executeBackward()
            else:
                print 'Please teleop back'
                raw_input('done?')
                
                
        
        calibBins.append(calibPts)
        
        filename = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/shelf_calib_data/shelf_calib_all_Final.json'
        with open(filename, 'w') as outfile:
            json.dump(calibBins, outfile)
        
        print 'planning to go home'
        home_joint_pose = [-0.005744439031, -0.6879946105, 0.5861570764, 0.09693312715, 0.1061231983, -0.1045031463]
        planner = IKJoint(target_joint_pos=home_joint_pose)
        plan = planner.plan()
        plan.visualize()
        raw_input('execute?')
        ik.ik.setSpeedByName('faster')
        plan.execute()
        print 'done going home'
    
        
    print 'dumped to file ', filename

if __name__=='__main__':
    rospy.init_node('listener', anonymous=True)
    
    # available options:
    # mouth, home, shelf
    manual_cal_2(binNum = 10, endBinNum = 11)
