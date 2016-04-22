#!/usr/bin/env python
# goToBin primitive:

# description: 
# called when we want to place object into objective bin

# inputs:
# configuration of the robot when called, assumed to be at the mouth bin
# location of the bin, xyz in world coordinate frame (center of bin), this
# does not need to be super accurate

import suction
import geometry_msgs.msg
import std_msgs
import json
import tf
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
from ik.ik import IKJoint
import rospy
import pdb
import numpy as np
import math
import os
import tf.transformations as tfm
from manual_fit.srv import *
import gripper
from pr_msgs.msg import *
from ik.ik import Plan
import rospy
import sensor_msgs.msg
import visualization_msgs.msg
import trajectory_msgs.msg
import moveit_msgs.msg
import telnetlib
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
import time
import socket
from visualization_msgs.msg import *
from ik.helper import find_object_pose_type
import sys
import std_msgs.msg
import gripper
# put shared function into ik.helper module
from ik.helper import getBinMouthAndFloor
from ik.roshelper import coordinateFrameTransform
from ik.roshelper import pubFrame
from ik.helper import pauseFunc
from ik.helper import visualizeFunc
from ik.helper import getObjCOM
from ik.helper import openGripper
from ik.helper import closeGripper

def goToBin(binNum = 0,
            robotConfig = None,
            objectiveBinPos = [1.2,0,0.6], 
            isExecute = True,
            withPause = False,
            withSuction = False,
            counter = 0):
    ## objPose: world position and orientation frame attached to com of object in quaternion form. XYZ
    ## objId: object identifier
    ## robotConfig: current time robot configuration
    ## shelfPosition: shelf position in world frame
    ## force threshold: amount fo force needed to say we have a grasp
    planSuctionSuccess = False
    planGraspSuccess = False
    
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
    # rospy.sleep(0.1)
    # br.sendTransform(tuple(tip_hand_transform[0:3]), tfm.quaternion_from_euler(*tip_hand_transform[3:6]), rospy.Time.now(), 'tip', "link_6")
    # rospy.sleep(0.1)
    
    pubFrame(br, pose=tip_hand_transform, frame_id='target_pose', parent_frame_id='map', npub=1)
    
    # get position of the tcp in world frame
    pose_world = coordinateFrameTransform(tip_hand_transform[0:3], 'link_6', 'map', listener)
    tcpPos=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    tcpPosHome = tcpPos
    # set scoop orientation (rotate wrist)
    if not withSuction:
        gripperOri = [0, 0.7071, 0, 0.7071]
    if withSuction:
        print '[goToBin] with suck-down primitive, defining new end orientation of gripper'
        gripperOri = [pose_world.pose.orientation.x,pose_world.pose.orientation.y,pose_world.pose.orientation.z,pose_world.pose.orientation.w]
    
    # move back 10 cms to avoid object with shelf collision
    distAwayFromBin = 0.1
    targetPosition = [tcpPos[0]-distAwayFromBin, tcpPos[1], tcpPos[2]]
    q_initial = robotConfig
    planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = gripperOri, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
    plan = planner.plan()
    s = plan.success()
    if s:
        print '[goToBin] move away from bin by %d cm successful' % distAwayFromBin
        print '[goToBin] tcp at:', targetPosition
        plan.visualize()
        plans.append(plan)
        if isExecute:
            pauseFunc(withPause)
            plan.execute()
    else:
        print '[goToBin] move away from bin fail'
        return False
        
    qf = plan.q_traj[-1]
    
    if binNum > 8:
        #plan = Plan()
        #plan.q_traj = [[0, -0.3959, 0.58466, 0.03,   0.1152, -0.1745]]  # should use IKJoint
        planner = IKJoint(q0 = qf, target_joint_pos=[0, -0.3959, 0.58466, 0.03,   0.1152, -0.1745])
        plan = planner.plan()
        
        print '[goToBin] going home because risky to go straight to objective bin'
        plan.visualize()
        qf = plan.q_traj[-1]
        if isExecute:
            pauseFunc(withPause)
            plan.execute()
            
    # move above objective bin, approx
    use_JointPos = False
    if use_JointPos:
        planner = IKJoint(q0 = qf, target_joint_pos=[0, -0.2953, 0.4462, 0,   0.8292, 1.5707])
        plan = planner.plan()
        #plan = Plan()
        #plan.q_traj = [[0, -0.2953, 0.4462, 0,   0.8292, 1.5707]]  # should use IKJoint
        s = True
    else:
        if not withSuction:
            gripperOri = [0.7071,0.7071,0,0]
            locationAboveBinCOM = [0,0,0.15] # define over the lip distance in shelf frame
            targetPosition = [objectiveBinPos[0]+locationAboveBinCOM[0],objectiveBinPos[1]+locationAboveBinCOM[1],objectiveBinPos[2]+locationAboveBinCOM[2]]
            q_initial = qf
            planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = gripperOri, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
            plan = planner.plan()
            s = plan.success()
            if s:
                print '[goToBin] move to above bin success'
                planSuctionSuccess = True
                plans.append(plan)
                plan.visualize()
                qf = plan.q_traj[-1]
                if isExecute:
                    pauseFunc(withPause)
                    plan.execute()
            else:
                return False
                    
        if withSuction:
            bin_xyz, baseHeight = getBinMouthAndFloor(0.20, 10)
            bin_xyz = coordinateFrameTransform(bin_xyz, 'shelf', 'map', listener)
            targetPosition = [bin_xyz.pose.position.x,bin_xyz.pose.position.y,bin_xyz.pose.position.z]
            q_initial = qf
            planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = gripperOri, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
            plan = planner.plan()
            s = plan.success()
            print '[goToBin] setting joints for suction'
            while True:
                APCrobotjoints = ROS_Wait_For_Msg(joint_topic, sensor_msgs.msg.JointState).getmsg() 
                q0 = APCrobotjoints.position
                if len(q0) < 6:
                    continue
                else:
                    break
            if q0[5]<0:
                stackAngle = -(counter-1)*1.6*3.1415/180+9*3.1415/180
                possible_start_config=[stackAngle, 0.610, 1.0277, 0.0, -1.4834, 3.1415-2*math.pi]
            else:
                stackAngle = -(counter-1)*1.6*3.1415/180+9*3.1415/180
                possible_start_config=[stackAngle, 0.610, 1.0277, 0.0, -1.4834, 3.1415]
                
            # plan_n = Plan()
            # plan_n.q_traj=[possible_start_config]
            planner = IKJoint(target_joint_pos=possible_start_config)
            plan_n = planner.plan()
            plan_n.visualize()
            qf = plan_n.q_traj[-1]
            if isExecute:
                pauseFunc(withPause)
                plan_n.execute()
    
    
    ## go down and release object
    if not withSuction:
        q_initial = qf
        objectiveBinPos[1] = (counter-1)*0.036-0.2
        targetPosition = [objectiveBinPos[0],objectiveBinPos[1],objectiveBinPos[2]]
        planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = gripperOri, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        s = plan.success()
        if s:
            print '[goToBin] go to xyz of bin with vertical bias success'
            plans.append(plan)
            plan.visualize()
            if isExecute:
                pauseFunc(withPause)
                plan.execute()
        else:
            print '[goToBin] go to xyz of bin with vertical bias fail'
        qf = plan.q_traj[-1]
        openGripper()
    
    ## open gripper fully
    # rospy.sleep(0.5)
    # openGripper()
    rospy.sleep(0.5)
    suction.stop()
    return True

if __name__=='__main__':
    rospy.init_node('listener', anonymous=True)
    goToBin(binNum = 0,
            robotConfig = None,
            objectiveBinPos = [1.2,0,0.6], 
            isExecute = True,
            withPause = False,
            withSuction = False,
            counter = 0)

# objPose =  [1.60593056679, 0.29076179862, 0.863177359104], binNum = 3
# objPose =  [1.55620419979, 0.281148612499, 1.14214038849], binNum = 0, 
# obJPose =  [1.62570548058, 0.289612442255, 0.648919522762], binNum = 6,
