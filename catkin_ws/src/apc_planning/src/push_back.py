# Push the object to the back of the bin (helpful for multiple object case)

import geometry_msgs
import std_msgs
import json
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
from ik.ik import EvalPlan
import tf.transformations as tfm
import rospy
import pdb
import sys
import tf
import math
import numpy as np
import tf.transformations as tfm
from numpy import linalg as la
import itertools
import gripper
import os
from pr_msgs.msg import *
from manual_fit.srv import *
import time
import sensor_msgs.msg
from copy import deepcopy
import flush_grasp

# put shared function into ik.helper module
from ik.helper import find_object_pose_type
from ik.helper import getBinMouthAndFloor
from ik.helper import find_shelf_walls
from ik.helper import pauseFunc
from ik.helper import get_obj_dim
from ik.helper import graspGripper
from ik.roshelper import coordinateFrameTransform
from ik.helper import matrix_from_xyzquat
from ik.helper import quat_from_matrix
from ik.helper import rotmatY
from ik.helper import rotmatX
from ik.roshelper import coordinateFrameTransform
from ik.roshelper import pubFrame
from ik.helper import get_bin_inner_cnstr
from ik.helper import moveGripper

def push_back(objPose=[1.95,0.25,1.4,0,0,0,1],
            binNum=4,
            objId = 'crayola_64_ct',
            bin_contents = ['paper_mate_12_count_mirado_black_warrior','crayola_64_ct','expo_dry_erase_board_eraser'],
            robotConfig = None,
            grasp_range_lim=0.11,
            fing_width=0.06,
            isExecute = True,
            withPause = True):
    
    #~ *****************************************************************
    obj_dim=get_obj_dim(objId)
    obj_dim=np.array(obj_dim)
    #~ *****************************************************************

    # # DEFINE HAND WIDTH#########################
    hand_width=0.186
    
    #~ *****************************************************************
    ## initialize listener rospy
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    
    (shelf_position, shelf_quaternion) = lookupTransform("map", "shelf", listener)
    # print  shelf_position, shelf_quaternion 
    
    #~ *****************************************************************

    # Convert xyx quat to tranformation matrix for Shelf frame
    #~ shelf_pose_tfm_list=matrix_from_xyzquat(shelfPose[0:3], shelfPose[3:7])

    shelf_pose_tfm_list=matrix_from_xyzquat(shelf_position,shelf_quaternion)
    shelf_pose_tfm=np.array(shelf_pose_tfm_list)

    shelf_pose_orient=shelf_pose_tfm[0:3,0:3]
    #~ print '[PushBack] shelf_pose_orient'
    #~ print shelf_pose_orient
    
    shelf_pose_pos=shelf_pose_tfm[0:3,3]
    #~ print '[PushBack] shelf_pose_pos'
    #~ print shelf_pose_pos
        
    #Normalized axes of the shelf frame
    shelf_X=shelf_pose_orient[:,0]/la.norm(shelf_pose_orient[:,0])
    shelf_Y=shelf_pose_orient[:,1]/la.norm(shelf_pose_orient[:,1])
    shelf_Z=shelf_pose_orient[:,2]/la.norm(shelf_pose_orient[:,2])
    
    #~ *****************************************************************
    
    # Convert xyx quat to tranformaation matrix for Object frame
    
    obj_pose_tfm_list=matrix_from_xyzquat(objPose[0:3], objPose[3:7])
    obj_pose_tfm=np.array(obj_pose_tfm_list)
    
    obj_pose_orient=obj_pose_tfm[0:3,0:3]
    obj_pose_pos=obj_pose_tfm[0:3,3]
    #~ print '[PushBack] obj_pose_orient'
    #~ print obj_pose_orient
    
    #Normalized axes of the object frame
    obj_X=obj_pose_orient[:,0]/la.norm(obj_pose_orient[:,0])
    #~ print '[PushBack] obj_X'
    #~ print obj_X
    
    obj_Y=obj_pose_orient[:,1]/la.norm(obj_pose_orient[:,1])
    #~ print '[PushBack] obj_Y'
    #~ print obj_Y
    
    obj_Z=obj_pose_orient[:,2]/la.norm(obj_pose_orient[:,2])
    #~ print '[PushBack] obj_Z'
    #~ print obj_Z
    
    #Normalized object frame
    obj_pose_orient_norm=np.vstack((obj_X,obj_Y,obj_Z))
    obj_pose_orient_norm=obj_pose_orient_norm.transpose()
    #~ print '[PushBack] obj_pose_orient_norm'
    #~ print obj_pose_orient_norm
    
    #~ *****************************************************************
    
    # We will assume that hand normal tries to move along object dimension along the Y axis of the shelf (along the lenght of the bin)
    # Find projection of object axes on Y axis of the shelf frame
    
    proj_vecY=np.dot(shelf_Y,obj_pose_orient_norm)
    #~ print '[PushBack] proj'
    #~ print proj_vecY
    
    max_proj_valY,hand_norm_dir=np.max(np.fabs(proj_vecY)), np.argmax(np.fabs(proj_vecY))
    
    if proj_vecY[hand_norm_dir]>0:
        hand_norm_vec=-obj_pose_orient_norm[:,hand_norm_dir]
    else:
        hand_norm_vec=obj_pose_orient_norm[:,hand_norm_dir]
        
    #~ print '[PushBack] hand_norm_vec'
    #~ print hand_norm_vec
    
    #Find angle of the edge of the object 
    Cos_angle_made_with_shelf_Y=max_proj_valY/(la.norm(shelf_Y)*la.norm(hand_norm_vec));
                        
    angle_to_shelfY=np.arccos(Cos_angle_made_with_shelf_Y)*180/np.pi
    
    #~ print'angle_to_shelfY'
    #~ print angle_to_shelfY
    #Find object dimension along hand normal axis
            
    obj_dim_along_hand_norm=obj_dim[hand_norm_dir]
    # print '[PushBack] dim along hand norm'
    # print obj_dim_along_hand_norm
    #~ *****************************************************************
    
    #Find projection of object axes on X axis of the shelf frame    
    #To find out which object frame is lying closer to the X axis of the shelf
        
    proj_vecX=np.dot(shelf_X,obj_pose_orient_norm)
        
    max_proj_valX,fing_axis_dir=np.max(np.fabs(proj_vecX)), np.argmax(np.fabs(proj_vecX))
    
    if proj_vecX[fing_axis_dir]>0:
        fing_axis_vec=obj_pose_orient_norm[:,fing_axis_dir]
    else:
        fing_axis_vec=-obj_pose_orient_norm[:,fing_axis_dir]
        
    #~ print '[PushBack] fing_axis_vec'
    #~ print fing_axis_vec
        
    #Find object dimension along the finger axis
    obj_dim_along_fingAxis=obj_dim[fing_axis_dir]
    # print '[PushBack] obj_dim_along_fingAxis'
    # print obj_dim_along_fingAxis
    
    #~ *****************************************************************
    
    #Find projection of object axes on Z axis of the shelf frame    
    #To find out which object frame is lying closer to the Z axis of the shelf
        
    proj_vecZ=np.dot(shelf_Z,obj_pose_orient_norm)

    max_proj_valZ,Zaxis_dir=np.max(np.fabs(proj_vecZ)), np.argmax(np.fabs(proj_vecZ))

    Zaxis_vec=obj_pose_orient_norm[:,Zaxis_dir]
        
    #~ print '[PushBack] Zaxis_vec'
    #~ print Zaxis_vec
    #Find object dimension along the finger axis, shelf Z
    obj_dim_along_ZAxis=obj_dim[Zaxis_dir]
    
    hand_Y=np.cross(hand_norm_vec,fing_axis_vec)
    
    diag_dist=la.norm(np.array([obj_dim_along_hand_norm,obj_dim_along_fingAxis]))
    # print'req obj diag dist=', diag_dist
    
    #~ *****************************************************************
    # Find the maximum distance to which we will push
    
    other_diag=np.zeros(len(bin_contents))
    
    for num_bin_contents in range(0, len(bin_contents)):
        if bin_contents[num_bin_contents] != objId:
            temp_obj_dim=get_obj_dim(objId)
            other_diag[num_bin_contents]=la.norm(np.array([temp_obj_dim[0],temp_obj_dim[1]]))
    
    max_diag,max_obj_ID=np.max(np.fabs(other_diag)), np.argmax(np.fabs(other_diag))
    # print '[PushBack] max_obj_ID',max_diag
    # print '[PushBack] max_obj_ID', bin_contents[max_obj_ID]
    #*******************************************************************
    
    bin_inner_cnstr=get_bin_inner_cnstr()
    
    #*******************************************************************
    #Check if we can do left push
    diag_factor=0.75
    
    obj_left_edge=obj_pose_pos[1]+diag_factor*diag_dist/2.0
    
    binLeftWall=bin_inner_cnstr[binNum][1]
    binLeftWall_world = coordinateFrameTransform([binLeftWall,0,0], 'shelf', 'map', listener)
    
    binRightWall=bin_inner_cnstr[binNum][0]
    binRightWall_world = coordinateFrameTransform([binRightWall,0,0], 'shelf', 'map', listener)
    
    left_gap=binLeftWall_world.pose.position.y-obj_left_edge
    # print '[PushBack] left_gap=', left_gap
    
    fing_clearance=0.010 # It's like 5 mm on both sides of finger
    
    bin_width=binLeftWall_world.pose.position.y-binRightWall_world.pose.position.y
    
    tolerance_in_push=0.020
    if left_gap+diag_dist/2.0-tolerance_in_push>bin_width:
        print '[PushBack] looks like vision has messed up. the object does not seem to be in the appropriate bin'
        return (False,False)
            
    if left_gap > fing_width+fing_clearance:
        left_push=True
        print '[PushBack] left push is possible'
    else:
        left_push=False
        print '[PushBack] left push is NOT possible'
        
    # check if we can do right push 
    
    obj_right_edge=obj_pose_pos[1]-diag_factor*diag_dist/2.0
    
    right_gap=obj_right_edge-binRightWall_world.pose.position.y
    # print '[PushBack] right_gap', right_gap
    fing_clearance=0.010 # It's like 5 mm on both sides of finger
    
    if right_gap+diag_dist/2.0-tolerance_in_push>bin_width:
        print '[PushBack] looks like vision has messed up. the object does not seem to be in the appropriate bin'
        return (False,False)
        
    if right_gap > fing_width+fing_clearance:
        right_push=True
        print '[PushBack] right push is possible'
    else:
        right_push=False
        print '[PushBack] right push is NOT possible'
    #*******************************************************************
    #Find Push Points for left push
    bin_face_pos,binFloorHeight=getBinMouthAndFloor(0.0, binNum)
    bin_face_pos_world = coordinateFrameTransform(bin_face_pos, 'shelf', 'map', listener)
    
    tcp_Z_off=0.005 #lower down from center of the bin
    push_tcp_Z_shelfFrame=((bin_inner_cnstr[binNum][4]+bin_inner_cnstr[binNum][5])/2.0)-tcp_Z_off
    
    push_tcp_Z_WorldFrame = coordinateFrameTransform([0,0,push_tcp_Z_shelfFrame], 'shelf', 'map', listener)
    
    push_tcp_Z=push_tcp_Z_WorldFrame.pose.position.z
    
    push_side_off=1.0
    if left_push:
        
        left_pushPt_X=bin_face_pos_world.pose.position.x+0.05
        left_pushPt_Y=binLeftWall_world.pose.position.y-push_side_off*left_gap
        # left_Y_lim=binLeftWall_world.pose.position.y-
        left_pushPt_Z=deepcopy(push_tcp_Z)
        
        left_push_Pt=np.array([left_pushPt_X,left_pushPt_Y,left_pushPt_Z])
        print '[PushBack] expected left push clearance=',binLeftWall_world.pose.position.y-left_pushPt_Y-fing_width/2.0
    #Find Push Points for right push
    
    if right_push:
        
        right_pushPt_X=bin_face_pos_world.pose.position.x+0.05
        right_pushPt_Y=binRightWall_world.pose.position.y+push_side_off*right_gap
        right_pushPt_Z=deepcopy(push_tcp_Z)
        
        
        right_push_Pt=np.array([right_pushPt_X,right_pushPt_Y,right_pushPt_Z])        
        
        print '[PushBack] expected right push clearance=',right_pushPt_Y-binRightWall_world.pose.position.y-fing_width/2.0
    #Execute push trajectories
    
    #~ *************************************************************
    # set spatual down orientation (rotate wrist)
    spatula_down_orient = [0, 0.7071, 0, 0.7071]
    
    push_orient=spatula_down_orient
    
    push_hand_opening=0.100
    
    joint_topic = '/joint_states'
    #~ *************************************************************
    
    #Execure left push trajectory
    
    if left_push:
        # plan store
        left_plans = []
        
        # set tcp (same as that set in generate dictionary)
        l1=0.0
        l2=0.0
        l3 = 0.47  
        tip_hand_transform = [l1, l2, l3, 0,0,0] # to be updated when we have a hand design finalized
        # broadcast frame attached to tcp
        pubFrame(br, pose=tip_hand_transform, frame_id='tip', parent_frame_id='link_6', npub=5)
        
        #~ *************************************************************
        
        # GO TO MOUTH OF THE BIN
        
        q_initial = robotConfig
        # move to bin mouth
        distFromShelf = 0.1
        mouthPt,temp = getBinMouthAndFloor(distFromShelf, binNum)
        mouthPt = coordinateFrameTransform(mouthPt, 'shelf', 'map', listener)
        targetPosition = [mouthPt.pose.position.x, mouthPt.pose.position.y, mouthPt.pose.position.z]
        gripperOri=[0, 0.7071, 0, 0.7071]
        
        pubFrame(br, pose = targetPosition+gripperOri, frame_id='target_pose', parent_frame_id='link_6', npub=5)

        planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = gripperOri, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        plan.setSpeedByName('faster')
        s = plan.success()
        if s:
            print '[PushBack] move to bin mouth in push-back code successful (left push)'
            left_plans.append(plan) # Plan 0
        else:
            print '[PushBack] move to bin mouth in push-back code fail (left push)'
            return (False,False)
        
        qf = plan.q_traj[-1]
    
        q_initial = qf
        #################### CLOSE THE GRIPPER GRASP #################
        grasp_plan = EvalPlan('moveGripper(%f, 100)' % (0.0))
        left_plans.append(grasp_plan)
        #~ *************************************************************
        
        ###### MOVE THE ROBOT INSIDE THE BIN WITH PUSH ORIENTATION ######

        # set push_tcp
        push_l1=0.0
        push_l2=0.0
        
        push_l3 = 0.47  
        push_tip_hand_transform = [l1, l2, l3, 0,0,0] # to be updated when we have a hand design finalized
        # broadcast frame attached to grasp_tcp
        pubFrame(br, pose=push_tip_hand_transform, frame_id='push_tip', parent_frame_id='link_6', npub=10)
        
        distFromShelf = -0.015
        InbinPt,temp = getBinMouthAndFloor(distFromShelf, binNum)
        InbinPt = coordinateFrameTransform(InbinPt, 'shelf', 'map', listener)
        prepush_targetPosition = [InbinPt.pose.position.x, left_push_Pt[1] , InbinPt.pose.position.z] #matching world_Y of first push pt
        
        pubFrame(br, pose=prepush_targetPosition+push_orient, frame_id='target_pose', parent_frame_id='map', npub=10)

        planner = IK(q0 = q_initial, target_tip_pos = prepush_targetPosition, target_tip_ori = push_orient, tip_hand_transform=push_tip_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        s = plan.success()
        plan.setSpeedByName('fast')
        s = plan.success()
        if s:
            print '[PushBack] move inside bin in push orient successful'
            #~ print '[PushBack] tcp at:'
            #~ print(pregrasp_targetPosition)
            #~ plan.visualize()
            left_plans.append(plan) # Plan 1
            #~ if isExecute:
                #~ pauseFunc(withPause)
                #~ plan.execute()
        else:
            print '[PushBack] move inside bin in push orient fail'
            return (False,False)
        
        qf = plan.q_traj[-1]
    
        q_initial = qf
        
        #~ *************************************************************
        
        ############## OPEN THE GRIPPER To PUSH###############
        
        # Open the gripper to dim calculated for pushing
        print '[PushBack] hand opening to'
        print push_hand_opening*1000.0
        grasp_plan = EvalPlan('moveGripper(%f, 100)' % (push_hand_opening))
        left_plans.append(grasp_plan)
        
        #~ *************************************************************
        
        ############  Execute the push trajectory ##############
        
        back_bin_X=bin_face_pos_world.pose.position.x+0.43
        push_stop_worldX=back_bin_X-max_diag-0.010

        left_pushStop_Pt_pos=deepcopy(left_push_Pt)
        left_pushStop_Pt_pos[0]=push_stop_worldX
        
        #Push until the end
    
        planner = IK(q0 = q_initial, target_tip_pos = left_pushStop_Pt_pos, target_tip_ori = push_orient, 
             joint_topic=joint_topic, tip_hand_transform=push_tip_hand_transform)
        plan = planner.plan()
        plan.setSpeedByName('fast')
        s = plan.success()
        if s:
            print '[PushBack] start point push traj successful'
            left_plans.append(plan) # Plan 1
        else:
            print '[PushBack] start point push traj fail'
            return (False,False)
        
        qf = plan.q_traj[-1]
        q_initial = qf
        
        #################### CLOSE THE GRIPPER GRASP #################
        grasp_plan = EvalPlan('moveGripper(%f, 100)' % (0.0))
        left_plans.append(grasp_plan)
        
        #~ *************************************************************
        # #################### EXECUTE FORWARD #################### 
        
        for numOfPlan in range(0, len(left_plans)):
            if isExecute:
                left_plans[numOfPlan].visualize()
                pauseFunc(withPause)
                left_plans[numOfPlan].execute()
        
        # #################### RETREAT #################### 
        
        for numOfPlan in range(0, len(left_plans)):
            if isExecute:
                left_plans[numOfPlan].visualizeBackward()
                pauseFunc(withPause)
                left_plans[len(left_plans)-numOfPlan-1].executeBackward()
        
    #*******************************************************************
    
    if right_push: 
        # plan store
        right_plans = []
        
        # set tcp (same as that set in generate dictionary)
        l1=0.0
        l2=0.0
        l3 = 0.47  
        tip_hand_transform = [l1, l2, l3, 0,0,0] # to be updated when we have a hand design finalized
        # broadcast frame attached to tcp
        pubFrame(br, pose=tip_hand_transform, frame_id='tip', parent_frame_id='link_6', npub=5)
        
        #~ *************************************************************
        
        # GO TO MOUTH OF THE BIN
        
        q_initial = robotConfig
        # move to bin mouth
        distFromShelf = 0.1
        mouthPt,temp = getBinMouthAndFloor(distFromShelf, binNum)
        mouthPt = coordinateFrameTransform(mouthPt, 'shelf', 'map', listener)
        targetPosition = [mouthPt.pose.position.x, mouthPt.pose.position.y, mouthPt.pose.position.z]
        gripperOri=[0, 0.7071, 0, 0.7071]
        
        pubFrame(br, pose = targetPosition+gripperOri, frame_id='target_pose', parent_frame_id='link_6', npub=5)

        planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = gripperOri, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        plan.setSpeedByName('faster')
        s = plan.success()
        if s:
            print '[PushBack] move to bin mouth in push-back code successful (right push)'
            right_plans.append(plan) # Plan 0
        else:
            print '[PushBack] move to bin mouth in push-back code fail (right push)'
            return (False,False)
        
        qf = plan.q_traj[-1]
    
        q_initial = qf
        #################### CLOSE THE GRIPPER GRASP #################
        grasp_plan = EvalPlan('moveGripper(%f, 100)' % (0.0))
        right_plans.append(grasp_plan)
        #~ *************************************************************
        
        ###### MOVE THE ROBOT INSIDE THE BIN WITH PUSH ORIENTATION ######

        # set push_tcp
        push_l1=0.0
        push_l2=0.0
        
        push_l3 = 0.47  
        push_tip_hand_transform = [l1, l2, l3, 0,0,0] # to be updated when we have a hand design finalized
        # broadcast frame attached to grasp_tcp
        pubFrame(br, pose=push_tip_hand_transform, frame_id='push_tip', parent_frame_id='link_6', npub=10)
        
        distFromShelf = -0.015
        InbinPt,temp = getBinMouthAndFloor(distFromShelf, binNum)
        InbinPt = coordinateFrameTransform(InbinPt, 'shelf', 'map', listener)
        prepush_targetPosition = [InbinPt.pose.position.x, right_push_Pt[1] , InbinPt.pose.position.z] #matching world_Y of first push pt
        
        pubFrame(br, pose=prepush_targetPosition+push_orient, frame_id='target_pose', parent_frame_id='map', npub=10)

        planner = IK(q0 = q_initial, target_tip_pos = prepush_targetPosition, target_tip_ori = push_orient, tip_hand_transform=push_tip_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        s = plan.success()
        plan.setSpeedByName('fast')
        s = plan.success()
        if s:
            print '[PushBack] move inside bin in push orient successful'
            #~ print '[PushBack] tcp at:'
            #~ print(pregrasp_targetPosition)
            #~ plan.visualize()
            right_plans.append(plan) # Plan 1
            #~ if isExecute:
                #~ pauseFunc(withPause)
                #~ plan.execute()
        else:
            print '[PushBack] move inside bin in push orient fail'
            return (False,False)
        
        qf = plan.q_traj[-1]
    
        q_initial = qf
        
        #~ *************************************************************
        
        ############## OPEN THE GRIPPER To PUSH###############
        
        # Open the gripper to dim calculated for pushing
        print '[PushBack] hand opening to'
        print push_hand_opening*1000.0
        grasp_plan = EvalPlan('moveGripper(%f, 100)' % (push_hand_opening))
        right_plans.append(grasp_plan)
        
        #~ *************************************************************
        
        ############  Execute the push trajectory ##############
        
        back_bin_X=bin_face_pos_world.pose.position.x+0.42
        push_stop_worldX=back_bin_X-max_diag-0.010

        right_pushStop_Pt_pos=deepcopy(right_push_Pt)
        right_pushStop_Pt_pos[0]=push_stop_worldX
        
        #Push until the end
    
        planner = IK(q0 = q_initial, target_tip_pos = right_pushStop_Pt_pos, target_tip_ori = push_orient, 
             joint_topic=joint_topic, tip_hand_transform=push_tip_hand_transform)
        plan = planner.plan()
        plan.setSpeedByName('fast')
        s = plan.success()
        if s:
            print '[PushBack] start point push traj successful'
            right_plans.append(plan) # Plan 1
        else:
            print '[PushBack] start point push traj fail'
            return (False,False)
        
        qf = plan.q_traj[-1]
        q_initial = qf
        
        #################### CLOSE THE GRIPPER GRASP #################
        grasp_plan = EvalPlan('moveGripper(%f, 100)' % (0.0))
        right_plans.append(grasp_plan)
        
        #~ *************************************************************
        # #################### EXECUTE FORWARD #################### 
        
        for numOfPlan in range(0, len(right_plans)):
            if isExecute:
                right_plans[numOfPlan].visualize()
                pauseFunc(withPause)
                right_plans[numOfPlan].execute()
        
        # #################### RETREAT #################### 
        
        for numOfPlan in range(0, len(right_plans)):
            if isExecute:
                right_plans[numOfPlan].visualizeBackward()
                pauseFunc(withPause)
                right_plans[len(right_plans)-numOfPlan-1].executeBackward()
    
    if right_push or left_push:
        push_possible=True
    else:
        push_possible=False
        
    return(push_possible, False)

########################################################################

# To test the function 
if __name__=='__main__':
    
    # We get object pose as position and quaternion array
    # We get shelf pose as position and quaternion array
    
    #obj_dim=[0.08,0.06,0.03]

    ##assume for now that we know the rotation matrix (derived from quaternion)

    #objPose=[0.5, 0.4, 0.3, 0, 0, 0, 1]
    ##obj_pose_orient=np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    ##obj_pose_pos=np.array([0.5, 0.4, 0.3])

    #shelfPose=[0.5, 0.35, 0.1, 0, 0, 0, 1]
    ##shelf_pose_orient=np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]])
    ##shelf_pose_pos=np.array([0.5, 0.35, 0.1])
    #grasp_range_lim=0.1
    #fing_width=0.06

    #bin_ID=1
    #off_perc=.2 # should depend on object ID
    
    rospy.init_node('listener', anonymous=True)
    # 
    # posesrv = rospy.ServiceProxy('/pose_service', GetPose)   # should move service name out
    # rospy.sleep(0.5)
    # data = posesrv('','')
    # pos_x= data.pa.object_list[1].pose.position.x
    # pos_y= data.pa.object_list[1].pose.position.y
    # pos_z= data.pa.object_list[1].pose.position.z
    # orient_x= data.pa.object_list[1].pose.orientation.x 
    # orient_y= data.pa.object_list[1].pose.orientation.y
    # orient_z= data.pa.object_list[1].pose.orientation.z
    # orient_w= data.pa.object_list[1].pose.orientation.w
    
    objPoses = []
    objPoses.append([1.599, 0.450, 1.106, -0.0030, 0.6979, -0.7161, 0.0023]) # upper left corner
    objPoses.append([1.607, -0.494, 1.130, -0.0030, 0.6979, -0.7161, 0.0023]) # uppper right corner
    objPoses.append([1.598, 0.467, 0.412, -0.0030, 0.6911, -0.7227, 0.0023]) # lower left corner
    objPoses.append([1.660, -0.500, 0.379, -0.0384, 0.6970, -0.7150, 0.0368]) #lower right corner
    
    binNums = [0,2,9,11]
    
    for x in range(0,4):
        (push_possible, execution_possible)=push_back(objPose = objPoses[x],
                binNum = binNums[x],
                objId = 'crayola_64_ct',
                bin_contents = ['paper_mate_12_count_mirado_black_warrior','crayola_64_ct','expo_dry_erase_board_eraser'], 
                robotConfig = None,
                grasp_range_lim=0.11,
                fing_width=0.06,
                isExecute = True,
                withPause = True)
