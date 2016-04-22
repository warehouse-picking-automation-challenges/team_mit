#!/usr/bin/env python
# Plan for push to rate the object so that it can be grasped

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
from ik.roshelper import lookupTransform
from ik.roshelper import pubFrame
from ik.helper import get_bin_inner_cnstr
from ik.helper import moveGripper

#def dotproduct(v1, v2):
    #return sum((a*b) for a, b in zip(v1, v2))

# def find_bin_base(bin_ID):
    # bin_base_height_shelf_frame=np.matrix('1.52393 1.52393 1.52393 1.29533 1.29533 1.29533 1.06673 1.06673 1.06673 0.800027 0.800027 0.800027')
    # return bin_base_height_shelf_frame[0,bin_ID]


def push_rotate(objPose=[1.95,0.25,1.4,0,0,0,1],
            binNum=4,
            objId = 'crayola_64_ct',
            bin_contents = None, 
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
    br = tf.TransformBroadcaster()
    rospy.sleep(0.2)
    
    (shelf_position, shelf_quaternion) = lookupTransform("map", "shelf", listener)
    # print  shelf_position, shelf_quaternion 
    
    #~ *****************************************************************

    # Convert xyx quat to tranformation matrix for Shelf frame
    #~ shelf_pose_tfm_list=matrix_from_xyzquat(shelfPose[0:3], shelfPose[3:7])

    shelf_pose_tfm_list=matrix_from_xyzquat(shelf_position,shelf_quaternion)
    shelf_pose_tfm=np.array(shelf_pose_tfm_list)

    shelf_pose_orient=shelf_pose_tfm[0:3,0:3]
    #~ print 'shelf_pose_orient'
    #~ print shelf_pose_orient
    
    shelf_pose_pos=shelf_pose_tfm[0:3,3]
    #~ print 'shelf_pose_pos'
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
    #~ print 'obj_pose_orient'
    #~ print obj_pose_orient
    
    #Normalized axes of the object frame
    obj_X=obj_pose_orient[:,0]/la.norm(obj_pose_orient[:,0])
    #~ print 'obj_X'
    #~ print obj_X
    
    obj_Y=obj_pose_orient[:,1]/la.norm(obj_pose_orient[:,1])
    #~ print 'obj_Y'
    #~ print obj_Y
    
    obj_Z=obj_pose_orient[:,2]/la.norm(obj_pose_orient[:,2])
    #~ print 'obj_Z'
    #~ print obj_Z
    
    #Normalized object frame
    obj_pose_orient_norm=np.vstack((obj_X,obj_Y,obj_Z))
    obj_pose_orient_norm=obj_pose_orient_norm.transpose()
    #~ print 'obj_pose_orient_norm'
    #~ print obj_pose_orient_norm
    
    #~ *****************************************************************
    
    # We will assume that hand normal tries to move along object dimension along the Y axis of the shelf (along the lenght of the bin)
    # Find projection of object axes on Y axis of the shelf frame
    
    proj_vecY=np.dot(shelf_Y,obj_pose_orient_norm)
    #~ print 'proj'
    #~ print proj_vecY
    
    max_proj_valY,hand_norm_dir=np.max(np.fabs(proj_vecY)), np.argmax(np.fabs(proj_vecY))
    
    if proj_vecY[hand_norm_dir]>0:
        hand_norm_vec=-obj_pose_orient_norm[:,hand_norm_dir]
    else:
        hand_norm_vec=obj_pose_orient_norm[:,hand_norm_dir]
        
    #~ print 'hand_norm_vec'
    #~ print hand_norm_vec
    
    #Find angle of the edge of the object 
    Cos_angle_made_with_shelf_Y=max_proj_valY/(la.norm(shelf_Y)*la.norm(hand_norm_vec));
    
    angle_to_shelfY=np.arccos(Cos_angle_made_with_shelf_Y)*180.0/np.pi
    
    #~ print'angle_to_shelfY'
    #~ print angle_to_shelfY
    #Find object dimension along hand normal axis
    
    obj_dim_along_hand_norm=obj_dim[hand_norm_dir]
    print '[PushRotate] dim along hand norm', obj_dim_along_hand_norm
    #~ *****************************************************************
    
    #Find projection of object axes on X axis of the shelf frame    
    #To find out which object frame is lying closer to the X axis of the shelf
        
    proj_vecX=np.dot(shelf_X,obj_pose_orient_norm)
        
    max_proj_valX,fing_axis_dir=np.max(np.fabs(proj_vecX)), np.argmax(np.fabs(proj_vecX))
    
    if proj_vecX[fing_axis_dir]>0:
        fing_axis_vec=obj_pose_orient_norm[:,fing_axis_dir]
    else:
        fing_axis_vec=-obj_pose_orient_norm[:,fing_axis_dir]
        
    #~ print 'fing_axis_vec'
    #~ print fing_axis_vec
        
    #Find object dimension along the finger axis
    obj_dim_along_fingAxis=obj_dim[fing_axis_dir]
    print '[PushRotate] obj_dim_along_fingAxis:', obj_dim_along_fingAxis
    
    #~ *****************************************************************
    
    #Find projection of object axes on Z axis of the shelf frame    
    #To find out which object frame is lying closer to the Z axis of the shelf
        
    proj_vecZ=np.dot(shelf_Z,obj_pose_orient_norm)

    max_proj_valZ,Zaxis_dir=np.max(np.fabs(proj_vecZ)), np.argmax(np.fabs(proj_vecZ))

    Zaxis_vec=obj_pose_orient_norm[:,Zaxis_dir]
        
    #~ print 'Zaxis_vec'
    #~ print Zaxis_vec
    #Find object dimension along the finger axis, shelf Z
    obj_dim_along_ZAxis=obj_dim[Zaxis_dir]
    
    hand_Y=np.cross(hand_norm_vec,fing_axis_vec)
    
    #~ *****************************************************************
    ####################  DECISION STRATEGIES #########################
    
    bin_inner_cnstr=get_bin_inner_cnstr()
    
    proj_handNorm_ShelfX=np.dot(hand_norm_vec,shelf_X)
    
    right_push=False
    left_push=False
    
    # print 'obj_dim_along_hand_norm', obj_dim_along_hand_norm
    # print 'obj_dim_along_fingAxis', obj_dim_along_fingAxis
    # 
    # print 'proj_handNorm_ShelfX'
    # print proj_handNorm_ShelfX
    
    if obj_dim_along_hand_norm>obj_dim_along_fingAxis:
        #~ print 'proj_vecY'
        #~ print proj_vecY
        #~ print proj_vecY[hand_norm_dir]
        #~ 
        #~ print 'proj_vecX'
        #~ print proj_vecX
        #~ print proj_vecX[fing_axis_dir]
        
        #~ if proj_vecY[hand_norm_dir]*proj_vecX[fing_axis_dir]<0:
            #~ #Do left push
            #~ Ypush_mult=1
            #~ print 'Left Push'
        #~ else:
            #~ #Do right push
            #~ Ypush_mult=-1
            #~ print 'right Push'
        if proj_handNorm_ShelfX>0:
            #Do left push
            Ypush_mult=1
            left_push=True
            print '[PushRotate] Left Push'
        else:
            #Do right push
            Ypush_mult=-1
            print '[PushRotate] right Push'
            right_push=True
        push_offset=(obj_dim_along_hand_norm/2.0) #(Instead pf pushing on edge we will push 5 mm inside)
            
    else:
        
        #~ if proj_vecY[hand_norm_dir]*proj_vecX[fing_axis_dir]<0:
            #~ #Do right push
            #~ Ypush_mult=-1
            #~ print 'right Push'
        #~ else:
            #~ #Do left push
            #~ Ypush_mult=1
            #~ print 'Left Push'
        if proj_handNorm_ShelfX>0:
            #Do right push
            Ypush_mult=-1
            print '[PushRotate] right Push'
            right_push=True
        else:
            #Do left push
            Ypush_mult=1
            print '[PushRotate] Left Push'
            left_push=True
        push_offset=(obj_dim_along_fingAxis/2.0)
    
    num_intm_points=5
    push_x_intm=np.zeros(num_intm_points+1)
    push_y_intm=np.zeros(num_intm_points+1)
    
    backWall_shelf=bin_inner_cnstr[binNum][1]
    backWall_world = coordinateFrameTransform([0,backWall_shelf,0], 'shelf', 'map', listener)
    
    back_check=backWall_world.pose.position.x-obj_dim_along_hand_norm/2.0
    
    
    binRightWall=bin_inner_cnstr[binNum][0]
    binLeftWall=bin_inner_cnstr[binNum][1]
        
    binRightWall_world = coordinateFrameTransform([binRightWall,0,0], 'shelf', 'map', listener)
    binLeftWall_world = coordinateFrameTransform([binLeftWall,0,0], 'shelf', 'map', listener)
    
    if left_push:
        sideWall_check=binRightWall_world.pose.position.y+(fing_width/2.0)+(obj_dim_along_fingAxis)
    
    if right_push:
        sideWall_check=binLeftWall_world.pose.position.y-(fing_width/2.0)-(obj_dim_along_fingAxis)
    
    for i in range(num_intm_points+1):
        push_x_intm[i]=obj_pose_pos[0]+(push_offset)*np.sin(((90/num_intm_points)*i*np.pi)/180.0)
        
        if push_x_intm[i]>back_check:
            push_x_intm[i]=back_check
        
        push_y_intm[i]=obj_pose_pos[1]+(Ypush_mult*push_offset)*np.cos(((90/num_intm_points)*i*np.pi)/180.0)
        
        if left_push:
            if push_y_intm[i]<sideWall_check:
                push_y_intm[i]=sideWall_check
                print '[PushRotate] push Y reduced, I do not want to crush the obj and gripper'
        
        if right_push:
            if push_y_intm[i]>sideWall_check:
                push_y_intm[i]=sideWall_check
                print '[PushRotate] push Y reduced, I do not want to crush the obj and gripper'
    
    push_x_intm=np.array(push_x_intm)
    
    #~ print'push_x_intm'
    #~ print push_x_intm
    
    push_y_intm=np.array(push_y_intm)
    
    #*******************************************************************
    #Move the hand to the center of the bin and open the hand based on the height of the object
    
    bin_mid_pos,binFloorHeight=getBinMouthAndFloor(0.0, binNum)
    binFloorHeight_world = coordinateFrameTransform([0,0,binFloorHeight], 'shelf', 'map', listener)
        
    bin_mid_pos_world = coordinateFrameTransform(bin_mid_pos, 'shelf', 'map', listener)
    
    #*******************************************************************
    tcp_Z_off=0.005
    push_tcp_Z_shelfFrame=((bin_inner_cnstr[binNum][4]+bin_inner_cnstr[binNum][5])/2.0)-tcp_Z_off
    
    push_tcp_Z_WorldFrame = coordinateFrameTransform([0,0,push_tcp_Z_shelfFrame], 'shelf', 'map', listener)
    
    push_tcp_Z=push_tcp_Z_WorldFrame.pose.position.z
    
    push_z_intm=(push_tcp_Z)*np.ones(push_x_intm.size)
    
    push_series_pos=np.vstack((push_x_intm,push_y_intm,push_z_intm))
    push_series_pos=push_series_pos.transpose()
    
    #~ print 'push_series_pos'
    #~ print push_series_pos
    
    #~ print push_series_pos[0,:]
    #~ print push_series_pos[-1,:]
    
    push_possible=True
    #******************************************************************
    fing_push_pt=0.2*obj_dim_along_ZAxis+binFloorHeight_world.pose.position.z
    
    blade_tip_TCP_off=0.038 # dist between finger inner end and spatual edge
    
    push_hand_opening=2*(push_tcp_Z-blade_tip_TCP_off-fing_push_pt)
    
    if push_hand_opening>grasp_range_lim:
        push_hand_opening=grasp_range_lim
    
        
    # Spatula finger should not hit the lip of the bin while pushing
    
    lip_Z_shelf=bin_inner_cnstr[binNum][4]
    lip_Z_world=coordinateFrameTransform([0,0,lip_Z_shelf], 'shelf', 'map', listener)
        
    #~ lip_off=.025
    #~ bin_lip_Z=binFloorHeight_world.pose.position.z+lip_off
    
    max_allowed_hand_opening_out=2*(push_tcp_Z-lip_Z_world.pose.position.z)
    max_allowed_hand_opening=max_allowed_hand_opening_out-0.036 # 0.036 is diff between inside and outside of the finger/finger mount surface
    
    if push_hand_opening>max_allowed_hand_opening:
        print '[PushRotate] reducing hand opening bcaz it may hit the lip of the bin'
        push_hand_opening=max_allowed_hand_opening
    
    
    #~ print 'push_hand_opening'
    #~ print push_hand_opening
        
    #~ #hand should not hit the side walls 
    #~ binRightWall,binLeftWall=find_shelf_walls(binNum)
    #~ 
    #~ binRightWall_world = coordinateFrameTransform([binRightWall,0,0], 'shelf', 'map', listener)
    #~ binLeftWall_world = coordinateFrameTransform([binLeftWall,0,0], 'shelf', 'map', listener)
    
    side_wall_clearance=0.0
    
    print '[PushRotate] binRightWall_world.pose.position.y', binRightWall_world.pose.position.y
    print '[PushRotate] right_hand_edge=', (push_y_intm[0]-fing_width-side_wall_clearance)
    
    print '[PushRotate] binLeftWall_world.pose.position.y', binLeftWall_world.pose.position.y
    print '[PushRotate] left_hand_edge=', (push_y_intm[0]+fing_width+side_wall_clearance)
    
    if push_y_intm[0]-fing_width-side_wall_clearance<binRightWall_world.pose.position.y:
        print '[PushRotate] Hand will hit the right wall, Push rotate not possible'
        push_possible=False
    if push_y_intm[0]+fing_width+side_wall_clearance>binLeftWall_world.pose.position.y:
        print '[PushRotate] Hand will hit the left wall, push rotate not possible'
        push_possible=False

    #~ *************************************************************
    # set spatual down orientation (rotate wrist)
    spatula_down_orient = [0, 0.7071, 0, 0.7071]
    
    push_orient=deepcopy(spatula_down_orient)
        
    #~ *************************************************************
        
    if push_possible==True:
        #Now we know where we wan to move TCP
        #Let's do robot motion planning now
        
        joint_topic = '/joint_states'
            
        # plan store
        plans = []
        
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
            print '[PushRotate] move to bin mouth in push-rotate code successful'
            #plan.visualize()
            plans.append(plan) # Plan 0
            #if isExecute:
            #    pauseFunc(withPause)
             #   plan.execute()
        else:
            print '[PushRotate] move to bin mouth in push-rotate code fail'
            return False
        
        qf = plan.q_traj[-1]
    
        q_initial = qf
        #################### CLOSE THE GRIPPER GRASP #################
        grasp_plan = EvalPlan('moveGripper(%f, 100)' % (0.0))
        plans.append(grasp_plan)
        #~ moveGripper(0.1,100)
        #~ *************************************************************
        
        ###### MOVE THE ROBOT INSIDE THE BIN WITH PUSH ORIENTATION ######

        # set push_tcp
        push_l1=0.0 #0.035
        push_l2=-Ypush_mult*(fing_width/2.0) # This should depend on the righ or left push conditions
        
        push_l3 = 0.47  
        push_tip_hand_transform = [l1, l2, l3, 0,0,0] # to be updated when we have a hand design finalized
        # broadcast frame attached to grasp_tcp
        pubFrame(br, pose=push_tip_hand_transform, frame_id='push_tip', parent_frame_id='link_6', npub=10)
                
        #~ q_initial = robotConfig
        # move just inside the bin with push orientation and open the hand suitable to push
        
        distFromShelf = -0.01
        InbinPt,temp = getBinMouthAndFloor(distFromShelf, binNum)
        InbinPt = coordinateFrameTransform(InbinPt, 'shelf', 'map', listener)
        prepush_targetPosition = [InbinPt.pose.position.x, push_series_pos[0,1] , InbinPt.pose.position.z] #matching world_Y of first push pt
        
        print '[PushRotate] prepush_targetPosition=', prepush_targetPosition
        
        pubFrame(br, pose=prepush_targetPosition+push_orient, frame_id='target_pose', parent_frame_id='map', npub=10)

        planner = IK(q0 = q_initial, target_tip_pos = prepush_targetPosition, target_tip_ori = push_orient, tip_hand_transform=push_tip_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        plan.setSpeedByName('slow')
        s = plan.success()
        if s:
            print '[PushRotate] move inside bin in push orient successful'
            #~ print 'tcp at:'
            #~ print(pregrasp_targetPosition)
            #~ plan.visualize()
            plans.append(plan) # Plan 1
            #~ if isExecute:
                #~ pauseFunc(withPause)
                #~ plan.execute()
        else:
            print '[PushRotate] move inside bin in push orient fail'
            return False
        
        qf = plan.q_traj[-1]
    
        q_initial = qf
        
        #~ *************************************************************
        
        ############## OPEN THE GRIPPER To PUSH ###############
        
        # Open the gripper to dim calculated for pushing
        print '[PushRotate] hand opening to'
        print push_hand_opening*1000.0
        #~ moveGripper(push_hand_opening,100)
        grasp_plan = EvalPlan('moveGripper(%f, 100)' % (push_hand_opening))
        plans.append(grasp_plan)
        
        #~ *************************************************************
        
        ############  Execute the push trajectory ##############
        #~ push_rotate_traj = Plan()
        #~ push_rotate_traj.q_traj = qf
        
        for i in range(0,push_x_intm.size):
            pushpt_pos=push_series_pos[i,:].transpose()
            pubFrame(br, pose=pushpt_pos.tolist()+push_orient, frame_id='target_pose', parent_frame_id='map', npub=10)
                
            #~ pdb.set_trace()
            # planner = IK(q0 = q_initial, target_tip_pos = graspPT_pose_pos, target_tip_ori = push_orient, 
                 # joint_topic=joint_topic, tip_hand_transform=tip_hand_transform, straightness = 0.3, inframebb = inframebb)
            planner = IK(q0 = q_initial, target_tip_pos = pushpt_pos, target_tip_ori = push_orient, 
                 joint_topic=joint_topic, tip_hand_transform=push_tip_hand_transform)
            plan = planner.plan()
            
            plan.setSpeedByName('slow')
            s = plan.success()
            if s:
                print '[PushRotate] point inside push traj successful'
                #~ print 'tcp at:'
                #~ print(pregrasp_targetPosition)
                #~ plan.visualize()
                plans.append(plan) # Plan 1
                #~ if isExecute:
                    #~ pauseFunc(withPause)
                    #~ plan.execute()
            else:
                print '[PushRotate] point inside push traj fail'
                return False
            
            qf = plan.q_traj[-1]
            q_initial = qf
        #~ *************************************************************
        
        #~ if isExecute:
            #~ pauseFunc(withPause)
            #~ push_rotate_traj.execute()
            
        #################### CLOSE THE GRIPPER GRASP #################
        #~ moveGripper(0.0,100)
        grasp_plan = EvalPlan('moveGripper(%f, 100)' % (0.0))
        plans.append(grasp_plan)
        #~ *************************************************************
        
        # #################### EXECUTE FORWARD #################### 
        
        for numOfPlan in range(0, len(plans)):
            if isExecute:
                plans[numOfPlan].visualize()
                pauseFunc(withPause)
                plans[numOfPlan].execute()
        
        # #################### RETREAT #################### 
        
        for numOfPlan in range(0, len(plans)):
            if isExecute:
                plans[len(plans)-numOfPlan-1].visualizeBackward()
                pauseFunc(withPause)
                plans[len(plans)-numOfPlan-1].executeBackward()
        
    #*******************************************************************
        
        print '[PushRotate] push_successful'
    
    return (push_possible,False)

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
    
    posesrv = rospy.ServiceProxy('/pose_service', GetPose)   # should move service name out
    rospy.sleep(0.5)
    data = posesrv('','')
    pos_x= data.pa.object_list[1].pose.position.x
    pos_y= data.pa.object_list[1].pose.position.y
    pos_z= data.pa.object_list[1].pose.position.z
    orient_x= data.pa.object_list[1].pose.orientation.x
    orient_y= data.pa.object_list[1].pose.orientation.y
    orient_z= data.pa.object_list[1].pose.orientation.z
    orient_w= data.pa.object_list[1].pose.orientation.w
    
    (push_possible,execution_possible)=push_rotate(objPose=[pos_x,pos_y,pos_z,orient_x,orient_y,orient_z,orient_w],
            binNum=4,
            objId = 'crayola_64_ct',
            bin_contents = None, 
            robotConfig = None,
            grasp_range_lim=0.11,
            fing_width=0.06,
            isExecute = True,
            withPause = True)
