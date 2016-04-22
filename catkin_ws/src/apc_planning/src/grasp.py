#!/usr/bin/env python

# Plan for parallel jaw grasping
# The code chekcs if the parallel jaw grasp is possible or not based on the object pose
# and if possible plans for it.

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
from ik.helper import setForceGripper
from ik.roshelper import coordinateFrameTransform
from ik.roshelper import lookupTransform

from ik.helper import matrix_from_xyzquat
from ik.helper import quat_from_matrix
from ik.helper import rotmatY
from ik.helper import rotmatX
from ik.helper import rotmatZ
from ik.roshelper import pubFrame
from ik.helper import get_bin_inner_cnstr

def grasp(objPose=[1.95,0.25,1.4,0,0,0,1],
            binNum=2,
            objId = 'cheezit_big_original',
            bin_contents = None, 
            robotConfig = None,
            grasp_or_not_ang_lim=20,
            grasp_tilt_angle=5.0,
            grasp_range_lim=0.11,
            fing_width=0.06,
            off_perc=0.0,
            isExecute = True,
            withPause = False):
                
    #bin_ID: num: id the ID of the bin from where we want to pick up the object
    #obj_dim : list : is the dimensions of the object, variable type : list
    #objPose : list : is a list of 7 numbers showing x y z and quaternion (x y z w)
    #shelfPose : list : is a list of 7 ......    
    #grasp_or_not_ang_lim : num: is the limit under which we will attempt grasping
    #grasp lim : num : max opening of hand ( We will assume it to be 105 to accomodate inaccuracy in vision)
    # fing_width : finger width
    # off_perc : percentage of length which we want to allow the robot to grasp at (CG-%dim to CG+%dim)
    
    joint_topic = '/joint_states'
    execution_possible=False
    try_flush_grasp=False
    cheezit_grasp=False
    try_partial_tilt=False
    cheezit_with_flushgrasp=False

    gripperOri=[0, 0.7071, 0, 0.7071]
    
    #~ Error
    # ##### Do not Run Grasp code####
    #~ Error
    #~ *****************************************************************
    obj_dim=get_obj_dim(objId)
    obj_dim=np.array(obj_dim)
    
    #~ *****************************************************************
    #Object grasp type
    obj_pose_type=find_object_pose_type(objId,objPose)

    # # DEFINE HAND WIDTH#########################
    hand_width=0.186
    hand_fing_disp_Xoffset=0.017
    hand_lift=0.025 #during grasping
    gripper_width=0.146
    # GET Bin Boundaries for collision detection
    
    bin_inner_cnstr=get_bin_inner_cnstr()
    #~ nrows = len(bin_inner_cnstr)
    #~ ncols = 3
    #~ *****************************************************************
    ## initialize listener rospy
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    # broadcast frame attached to object frame
    pubFrame(br, pose=objPose, frame_id='obj', parent_frame_id='map', npub=10)
    
    
    (shelf_position, shelf_quaternion) = lookupTransform("map", "shelf", listener)
    #~ print  shelf_position, shelf_quaternion 
    
    #~ *****************************************************************

    # Convert xyx quat to tranformation matrix for Shelf frame
    #~ shelf_pose_tfm_list=matrix_from_xyzquat(shelfPose[0:3], shelfPose[3:7])

    shelf_pose_tfm_list=matrix_from_xyzquat(shelf_position,shelf_quaternion)
    shelf_pose_tfm=np.array(shelf_pose_tfm_list)

    shelf_pose_orient=shelf_pose_tfm[0:3,0:3]
    #~ print '[Grasp] shelf_pose_orient'
    #~ print shelf_pose_orient
    
    shelf_pose_pos=shelf_pose_tfm[0:3,3]
    #~ print '[Grasp] shelf_pose_pos'
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
    #~ print '[Grasp] obj_pose_orient'
    #~ print obj_pose_orient
    

    #Normalized axes of the object frame
    obj_X=obj_pose_orient[:,0]/la.norm(obj_pose_orient[:,0])
    #~ print '[Grasp] obj_X'
    #~ print obj_X
    
    obj_Y=obj_pose_orient[:,1]/la.norm(obj_pose_orient[:,1])
    #~ print '[Grasp] obj_Y'
    #~ print obj_Y
    
    obj_Z=obj_pose_orient[:,2]/la.norm(obj_pose_orient[:,2])
    #~ print '[Grasp] obj_Z'
    #~ print obj_Z
    
    #Normalized object frame
    obj_pose_orient_norm=np.vstack((obj_X,obj_Y,obj_Z))
    obj_pose_orient_norm=obj_pose_orient_norm.transpose()
    #~ print '[Grasp] obj_pose_orient_norm'
    #~ print obj_pose_orient_norm
    
    #~ *****************************************************************
    
    # We will assume that hand normal tries to move along object dimension along the Y axis of the shelf (along the lenght of the bin)
    # Find projection of object axes on Y axis of the shelf frame
    
    proj_vecY=np.dot(shelf_Y,obj_pose_orient_norm)
    #~ print '[Grasp] proj'
    #~ print proj_vecY
    
    max_proj_valY,hand_norm_dir=np.max(np.fabs(proj_vecY)), np.argmax(np.fabs(proj_vecY))
    
    if proj_vecY[hand_norm_dir]>0:
        hand_norm_vec=-obj_pose_orient_norm[:,hand_norm_dir]
    else:
        hand_norm_vec=obj_pose_orient_norm[:,hand_norm_dir]
        
    #~ print '[Grasp] hand_norm_vec'
    #~ print hand_norm_vec
    
    #Find angle of the edge of the object 
    Cos_angle_made_with_shelf_Y=max_proj_valY/(la.norm(shelf_Y)*la.norm(hand_norm_vec))
                        
    angle_to_shelfY=np.arccos(Cos_angle_made_with_shelf_Y)*180/np.pi
    
    #~ print'angle_to_shelfY'
    #~ print angle_to_shelfY
    #Find object dimension along hand normal axis
            
    obj_dim_along_hand_norm=obj_dim[hand_norm_dir]
    #~ 
    test=np.dot(shelf_Y,obj_pose_orient_norm[:,hand_norm_dir])
    
    #~ *****************************************************************
    
    #Find projection of object axes on X axis of the shelf frame    
    #To find out which object frame is lying closer to the X axis of the shelf
        
    proj_vecX=np.dot(shelf_X,obj_pose_orient_norm)
        
    max_proj_valX,fing_axis_dir=np.max(np.fabs(proj_vecX)), np.argmax(np.fabs(proj_vecX))
    
    if proj_vecX[fing_axis_dir]>0:
        fing_axis_vec=obj_pose_orient_norm[:,fing_axis_dir]
    else:
        fing_axis_vec=-obj_pose_orient_norm[:,fing_axis_dir]
        
    #~ print '[Grasp] fing_axis_vec'
    #~ print fing_axis_vec
        
    #Find object dimension along the finger axis
    obj_dim_along_fingAxis=obj_dim[fing_axis_dir]
    #~ print '[Grasp] obj_dim_along_fingAxis'
    #~ print obj_dim_along_fingAxis
    
    #~ *****************************************************************
    
    #Find projection of object axes on Z axis of the shelf frame    
    #To find out which object frame is lying closer to the Z axis of the shelf
        
    proj_vecZ=np.dot(shelf_Z,obj_pose_orient_norm)
        
    max_proj_valZ,Zaxis_dir=np.max(np.fabs(proj_vecZ)), np.argmax(np.fabs(proj_vecZ))
    
    Zaxis_vec=obj_pose_orient_norm[:,Zaxis_dir]
        
    #~ print '[Grasp] Zaxis_vec'
    #~ print Zaxis_vec
    #Find object dimension along the finger axis, shelf Z
    obj_dim_along_ZAxis=obj_dim[Zaxis_dir]
    
    hand_Y=np.cross(hand_norm_vec,fing_axis_vec)
    
    grasp_tilt_frame=np.vstack((fing_axis_vec,hand_Y,hand_norm_vec)) # We arange hand normal, fing axis such that they match Z and X axis of the hand resp.
    grasp_tilt_frame=grasp_tilt_frame.transpose()
    #~ *****************************************************************
    
    if obj_pose_type=='upright':
        obj_upright=True
    else:
        obj_upright=False
    
    #~ *****************************************************************
    flush_grasp_possible = False
    
    # CHEKC IF FREE GRASP IS POSSIBLE
    #~ FREE GRASP = STRAIGHT  DIAGONAL GRASP
    diag_dist=obj_dim_along_hand_norm*np.sin((angle_to_shelfY*np.pi)/180.0)+obj_dim_along_fingAxis*Cos_angle_made_with_shelf_Y
    
    #~ print '[Grasp] obj_dim_along_hand_norm=', obj_dim_along_hand_norm
    #~ print '[Grasp] obj_dim_along_fingAxis=', obj_dim_along_fingAxis
    
    if objId =='feline_greenies_dental_treats' and obj_upright and obj_dim_along_hand_norm > obj_dim_along_fingAxis:
        obj_dim_along_ZAxis=0.100
        diag_dist=0.090
    
    if objId =='munchkin_white_hot_duck_bath_toy' and obj_upright:
        obj_dim_along_ZAxis=0.090
        diag_dist=0.090
    
    if objId =='first_years_take_and_toss_straw_cup' and obj_upright:
        obj_dim_along_ZAxis=0.150
        diag_dist=0.100
    
    #~ print '[Grasp] diag_dist'
    #~ print diag_dist
    diag_allowance=1.0
    #~ diag_dist=la.norm(np.array([obj_dim_along_hand_norm,obj_dim_along_fingAxis]))
    StGrasp_objlist = ['kong_duck_dog_toy', 'kong_sitting_frog_dog_toy', 'kyjen_squeakin_eggs_plush_puppies','kong_air_dog_squeakair_tennis_ball','dr_browns_bottle_brush']
    if (diag_allowance*diag_dist<grasp_range_lim) or (objId in StGrasp_objlist):
        free_grasp=True
        print '[Grasp] Straight grasp possible'
    else:
        free_grasp=False
        print '[Grasp] Straight grasp NOT possible'
    
    #~ *****************************************************************
    grasp_excess=1.35
    if objId =='crayola_64_ct':
        grasp_excess=1.5
    
    if free_grasp== True:
        print '[Grasp] Going for straight grasp!'
        grasp_possible=True # This will be set to False if we can not do the grasping collision-free
        angle_to_shelfY=0.0
        Cos_angle_made_with_shelf_Y=1.0
        hand_norm_vec=np.array([1,0,0])
        fing_axis_vec=np.array([0,1,0])
        hand_Y=np.array([0,0,1])        
        graspPT_pose_orient_wo_tilt=np.vstack((fing_axis_vec,hand_Y,hand_norm_vec)) # We arange hand normal, fing axis such that they match Z and X axis of the hand resp.
        graspPT_pose_orient_wo_tilt=graspPT_pose_orient_wo_tilt.transpose()
        
        obj_dim_along_hand_norm=obj_dim_along_hand_norm*Cos_angle_made_with_shelf_Y+obj_dim_along_fingAxis*np.sin((angle_to_shelfY*np.pi)/180.0)
        
        grasp_width_pregrasp=diag_allowance*grasp_excess*diag_dist
        if obj_dim_along_hand_norm<obj_dim_along_fingAxis:
            grasp_width=obj_dim_along_hand_norm
        else:
            grasp_width=obj_dim_along_fingAxis
    elif angle_to_shelfY<grasp_or_not_ang_lim and obj_dim_along_fingAxis < grasp_range_lim:
        grasp_possible=True # This will be set to False if we can not do the grasping collision-free
        print '[Grasp] Going for tilted hand grasp'
        grasp_width_pregrasp=grasp_excess*obj_dim_along_fingAxis
        grasp_width=obj_dim_along_fingAxis
        
        graspPT_pose_orient_wo_tilt=deepcopy(grasp_tilt_frame)
    else:
        #~ print 'angle_to_shelfY'
        #~ print angle_to_shelfY
        
        #~ print 'obj_dim_along_fingAxis'
        #~ print obj_dim_along_fingAxis
        
        print '[Grasp] Any grasp not possible!'
        grasp_possible=False
        
    if objId =='kong_duck_dog_toy' or objId =='kong_sitting_frog_dog_toy':
        obj_dim_along_ZAxis=0.030
        grasp_width_pregrasp=0.070
        grasp_width=0.0
    
    if objId =='kyjen_squeakin_eggs_plush_puppies':
        obj_dim_along_ZAxis=0.050
        grasp_width_pregrasp=0.110
        grasp_width=0.010
    
    if objId =='kong_air_dog_squeakair_tennis_ball':
        obj_dim_along_ZAxis=0.060
        grasp_width_pregrasp=0.090
        grasp_width=0.010
    
    if objId =='safety_works_safety_glasses':
        obj_dim_along_ZAxis=0.040
        grasp_width_pregrasp=0.110
        grasp_width=0.0
        
    if objId =='dr_browns_bottle_brush':
        obj_dim_along_ZAxis=0.045
        grasp_width_pregrasp=0.110
        grasp_width=0.010
    
    if objId =='champion_copper_plus_spark_plug':
        obj_dim_along_ZAxis=0.020
        #~ grasp_width_pregrasp=0.100
        grasp_width=0.010
        
    if objId =='first_years_take_and_toss_straw_cup' and obj_upright:
        #~ obj_dim_along_ZAxis=0.020
        grasp_width_pregrasp=0.110
        grasp_width=0.010
        
    if objId =='feline_greenies_dental_treats' and obj_upright:
        #~ obj_dim_along_ZAxis=0.020
        grasp_width_pregrasp=0.080
        grasp_width=0.010
        
    if objId =='munchkin_white_hot_duck_bath_toy' and obj_upright:
        obj_dim_along_ZAxis=0.100
        grasp_width_pregrasp=0.085
        grasp_width=0.010
    
    if objId =='crayola_64_ct':
        obj_dim_along_ZAxis=0.110
        off_perc=-0.15
    
    if objId =='mommys_helper_outlet_plugs' and obj_upright:
        obj_dim_along_ZAxis=0.100
    
    if objId =='expo_dry_erase_board_eraser' and obj_upright:
        obj_dim_along_ZAxis=0.065
        
    if grasp_possible:
        bin_mid_pos,binFloorHeight=getBinMouthAndFloor(0.0, binNum)
        binFloorHeight_world = coordinateFrameTransform([0,0,binFloorHeight], 'shelf', 'map', listener)
        
        if obj_dim_along_ZAxis<0.080:
            short_obj=True
            off_perc=0.2
            grasp_height_off=0.75
        else:
            short_obj=False
            grasp_height_off=0.55
        
        
        if grasp_width_pregrasp>grasp_range_lim:
            grasp_width_pregrasp=grasp_range_lim
        
        # Define the grasp Point
        graspPT_pose_pos=deepcopy(obj_pose_pos)
        graspPT_pose_pos[2]=binFloorHeight_world.pose.position.z+grasp_height_off*obj_dim_along_ZAxis
        
        #~ print 'graspPT_pose_pos=', graspPT_pose_pos
        #~ print 'obj_pose_pos=', obj_pose_pos
        
        graspDeep_vec_hand_frame=np.array([0,0,off_perc*obj_dim_along_hand_norm])
        graspDeep_vec_Rob_frame=np.dot(graspPT_pose_orient_wo_tilt,graspDeep_vec_hand_frame.transpose())
               
        graspDeep_pos=graspPT_pose_pos+graspDeep_vec_Rob_frame
        
        #~ print 'graspDeep_pos=', graspDeep_pos
        
        #Set Free grasp offset %
        freeGrasp_off_perc=0.25
        
        if free_grasp== True:
            if objId !='kong_duck_dog_toy' and objId !='kong_sitting_frog_dog_toy' and objId!='kyjen_squeakin_eggs_plush_puppies':
                print '[Grasp]  Not furry object'
                Freegraspoff_vec_hand_frame=np.array([0,0,freeGrasp_off_perc*obj_dim_along_hand_norm])
                Freegraspoff_vec_Rob_frame=np.dot(grasp_tilt_frame,graspDeep_vec_hand_frame.transpose())
                
                Freegraspoff_pos=graspDeep_pos-Freegraspoff_vec_Rob_frame
                
                graspDeep_pos[1]=Freegraspoff_pos[1] # This is to make sure that while free grasping we grasp offest
                                                    # such that we grasp towards to side coming out of the shelf        
        #~ print '[Grasp] graspDeep_pos_aft_off'
        #~ print graspDeep_pos

        #~ 
        # CHECK IF FINGER/HAND WILL HIT BIN LIP CONDITION !!!
        
        tcp_edge_off=0.12 # Distance between edge of the hand (flexible blade) and TCP
        
        #~ binbottomLipHeight=bin_inner_cnstr[binNum][4]
        
        fing_hit_dist=binFloorHeight_world.pose.position.z+(tcp_edge_off*np.sin((grasp_tilt_angle*np.pi)/180.0)+(fing_width/2.0)*np.cos((grasp_tilt_angle*np.pi)/180.0))
        
        bin_base_clearnace=0.004
        
        close_face_grasp=False
        
        if graspDeep_pos[2]<(fing_hit_dist+bin_base_clearnace):
            print '[Grasp] object very short, will try to pick up the object with tip of the fingers with increased tilt'
            close_face_grasp=True
            tcp_edge_off=0.043
            grasp_tilt_angle=9.0
        
        fing_hit_dist=binFloorHeight_world.pose.position.z+(tcp_edge_off*np.sin((grasp_tilt_angle*np.pi)/180.0)+(fing_width/2.0)*np.cos((grasp_tilt_angle*np.pi)/180.0))
        
        if graspDeep_pos[2]<fing_hit_dist+bin_base_clearnace:
            print '[Grasp] finger will hit the floor of the bin for ideal depth, checking if grasping is possible at min deep point'
            print '[Grasp] ********************'
            print '[Grasp] SLOW DOWN THE ROBOT'
            print '[Grasp] ********************'
            close_face_grasp=True
            graspDeep_pos[2]=fing_hit_dist+bin_base_clearnace #-0.020 ## ADJUST THIS FOR THIN OBJECTS
        
        #~ print '[Grasp] graspDeep_pos New'
        #~ print graspDeep_pos[2]
        #~ 
        lip_Z_shelf=bin_inner_cnstr[binNum][4]
        lip_Z_world=coordinateFrameTransform([0,0,lip_Z_shelf], 'shelf', 'map', listener)
        
        bin_mid_pos_world = coordinateFrameTransform(bin_mid_pos, 'shelf', 'map', listener)
        
        lip_clearance=(graspDeep_pos[0]-bin_mid_pos_world.pose.position.x)*np.tan((grasp_tilt_angle*np.pi)/180.0)-(fing_width/2.0)/np.cos((grasp_tilt_angle*np.pi)/180.0)
        
        #~ print '[Grasp] graspDeep_pos[2]+lip_clearance=', (graspDeep_pos[2]+lip_clearance)
        
        #~ print '[Grasp] lip_Z_world'
        #~ print lip_Z_world.pose.position.z
        
        steeper_grasp=False
        
        if (graspDeep_pos[2]+lip_clearance)<(lip_Z_world.pose.position.z+bin_base_clearnace):
            
            if close_face_grasp==True:
                print '[Grasp] Hand will hit the lip of the Bin and can not tilt the hand, so aborting grasp'
                grasp_possible=False
                
            else:
                print '[Grasp] Hand will hit the lip of the Bin, so trying more tilt'
                tcp_edge_off=0.08
                grasp_tilt_angle=15.0
                steeper_grasp=True
                lip_clearance=(graspDeep_pos[0]-bin_mid_pos_world.pose.position.x)*np.tan((grasp_tilt_angle*np.pi)/180.0)-(fing_width/2.0)/np.cos((grasp_tilt_angle*np.pi)/180.0)
                
                if (graspDeep_pos[2]+lip_clearance)<(lip_Z_world.pose.position.z+bin_base_clearnace):
                    print '[Grasp] Hand will still hit the lip of the Bin, so trying even steeper angle of tilt'
                    tcp_edge_off=0.055
                    grasp_tilt_angle=25.0
                
                    lip_clearance=(graspDeep_pos[0]-bin_mid_pos_world.pose.position.x)*np.tan((grasp_tilt_angle*np.pi)/180.0)-(fing_width/2.0)/np.cos((grasp_tilt_angle*np.pi)/180.0)
                    if (graspDeep_pos[2]+lip_clearance)<(lip_Z_world.pose.position.z+bin_base_clearnace):
                        print '[Grasp] Still the hand will hit the Lip of the Bin, so aborting grasp'
                        grasp_possible=False
                
        
        #~ print '[Grasp] obj_pose_pos'
        #~ print obj_pose_pos
        #~ 
        #~ print '[Grasp] graspDeep_pos'
        #~ print graspDeep_pos
    
        # CHECK HAND UpEdge HIT CONDITION !!!
        
        Uplip_Z_shelf=bin_inner_cnstr[binNum][5]
        Uplip_Z_world=coordinateFrameTransform([0,0,Uplip_Z_shelf], 'shelf', 'map', listener)
        #~ binUpperedge_Z_world=binFloorHeight_world.pose.position.z+2*(bin_mid_pos_world.pose.position.z-binFloorHeight_world.pose.position.z)
        
        hand_height=graspDeep_pos[2]+(bin_mid_pos_world.pose.position.x-graspDeep_pos[0])*np.tan((grasp_tilt_angle*np.pi)/180.0)+(fing_width/2.0)/np.cos((grasp_tilt_angle*np.pi)/180.0)
        #~ print '[Grasp] hand_height=', hand_height
        
        #~ print '[Grasp] Uplip_Z_world.pose.position.z=', Uplip_Z_world.pose.position.z
        
        if hand_height > Uplip_Z_world.pose.position.z:
            print '[Grasp] Hand will hit the top edge of the bin, so aborting grasp'
            grasp_possible=False
        
        # CHECK SIDE WALL HIT
        
        shelfLip_X_World=bin_mid_pos_world.pose.position.x
        #~ print '[Grasp] shelfLip_X_World'
        #~ print shelfLip_X_World
        
        hand_Zoffset_check=(graspDeep_pos[0]-shelfLip_X_World)/Cos_angle_made_with_shelf_Y
        
        #~ print '[Grasp] hand_Zoffset_check'
        #~ print hand_Zoffset_check
        
        hand_check_handFrame=np.array([0,0,hand_Zoffset_check])
        hand_check_worldFrame=np.dot(graspPT_pose_orient_wo_tilt,hand_check_handFrame.transpose())
        #~ 
        #~ print '[Grasp] grasp_width_pregrasp'
        #~ print grasp_width_pregrasp
        
        grasp_width_out=grasp_width_pregrasp+2*0.038
        if (bin_mid_pos_world.pose.position.x-graspDeep_pos[0])>0.220:
            hand_thick=0.148
            if grasp_width_out<hand_thick:
                grasp_width_out=hand_thick
                
        
        #~ print '[Grasp] grasp_width_out'
        #~ print grasp_width_out
        
        #~ hand_Yoffset_world_dist=((grasp_width_out/2.0)/np.fabs(Cos_angle_made_with_shelf_Y))+0.015 # This ideally should be 0.0
        
        #~ print '[Grasp] graspDeep_pos'
        #~ print graspDeep_pos
        
        #~ print '[Grasp] hand_check_worldFrame'
        #~ print hand_check_worldFrame
        
        hand_Yoffset_world_dist_left_fing=((grasp_width_out/2.0)/np.fabs(Cos_angle_made_with_shelf_Y))+0.017 ## MAY NEED TWEAK AT SEATTLE
        
        hand_Yoffset_world_dist_right_fing=((grasp_width_out/2.0)/np.fabs(Cos_angle_made_with_shelf_Y))+0.015 ## MAY NEED TWEAK AT SEATTLE
        
        hand_check_worldFrame_left=graspDeep_pos-hand_check_worldFrame
        hand_check_worldFrame_left[1]=hand_check_worldFrame_left[1]+hand_Yoffset_world_dist_left_fing-hand_fing_disp_Xoffset
        
        #~ print '[Grasp] hand_check_worldFrame_left'
        #~ print hand_check_worldFrame_left[1]
        
        hand_check_worldFrame_right=graspDeep_pos-hand_check_worldFrame
        hand_check_worldFrame_right[1]=hand_check_worldFrame_right[1]-hand_Yoffset_world_dist_right_fing-hand_fing_disp_Xoffset
        
        #~ print '[Grasp] hand_check_worldFrame_right'
        #~ print hand_check_worldFrame_right[1]
        
        #~ binRightWall,binLeftWall=find_shelf_walls(binNum)
        
        binRightWall=bin_inner_cnstr[binNum][0]
        binLeftWall=bin_inner_cnstr[binNum][1]
        
        binRightWall_world = coordinateFrameTransform([binRightWall,0,0], 'shelf', 'map', listener)
        binLeftWall_world = coordinateFrameTransform([binLeftWall,0,0], 'shelf', 'map', listener)
        
        #~ print '[Grasp] binRightWall_world.pose.position.y'
        #~ print binRightWall_world.pose.position.y
        
        if hand_check_worldFrame_right[1]<binRightWall_world.pose.position.y and grasp_possible==True:
                print '[Grasp] Hand will hit the right wall, so trying flush grasp'
                grasp_possible=False
                try_flush_grasp=True
                #~ flush_grasp_possible=flush_grasp(objPose = objPose,binNum = binNum,objId=objId,robotConfig=robotConfig,isExecute=isExecute,withPause=withPause)
        #~ 
        #~ print '[Grasp] binLeftWall_world.pose.position.y'
        #~ print binLeftWall_world.pose.position.y
        #~ 
        if hand_check_worldFrame_left[1]>binLeftWall_world.pose.position.y and grasp_possible==True:
            print '[Grasp] Hand will hit the left wall, so trying flush grasp'
            grasp_possible=False
            try_flush_grasp=True
            #~ flush_grasp_possible=flush_grasp(objPose = objPose,binNum = binNum,objId=objId,robotConfig=robotConfig,isExecute=isExecute,withPause=withPause)
                
    #~ *****************************************************************
    if objId =='cheezit_big_original' and obj_upright and grasp_possible:
        print '[Grasp] Cheezit is in graspable pose but upright, will execute special primitive'
        #~ grasp_possible=False
        cheezit_grasp=True
        
        partial_Z_angle=-45.0
        full_Z_angle=-75.0
        
        
    if grasp_possible:
        drake_success=True
        # Apply the hand tilt angle to orientation
        graspPT_pose_orient=np.dot(graspPT_pose_orient_wo_tilt,rotmatX(grasp_tilt_angle))
        #~ graspPT_pose_orient=graspPT_pose_orient_wo_tilt
        cheezit_graspPT_pose_orient_NOY=deepcopy(graspPT_pose_orient)
        graspPT_pose_orient_Rotmat=deepcopy(graspPT_pose_orient)
        
        #~ print '[Grasp] graspPT_pose_orient'
        #~ print graspPT_pose_orient
        
        graspPT_pose_orient = [[graspPT_pose_orient[0][0],graspPT_pose_orient[0][1],graspPT_pose_orient[0][2],0],
                               [graspPT_pose_orient[1][0],graspPT_pose_orient[1][1],graspPT_pose_orient[1][2],0],
                               [graspPT_pose_orient[2][0],graspPT_pose_orient[2][1],graspPT_pose_orient[2][2],0],
                               [0,0,0,1]]

        graspPT_pose_orient = np.array(graspPT_pose_orient)
        
        graspDeep_orient=quat_from_matrix(graspPT_pose_orient)
        
        cheezit_graspPT_pose_orient_quat=deepcopy(graspDeep_orient)
        
        # Grasp pose identified
        #~ print '[Grasp] graspDeep_pos'
        #~ print graspDeep_pos
        
        #~ print '[Grasp] graspDeep_orient'
        #~ print graspDeep_orient
        
        #~ *************************************************************
        #Now we know where we wan to move TCP
        #Let's do robot motion planning now
        
        
        # plan store
        plans = []
        
        # set tcp (same as that set in generate dictionary)
        l1=hand_fing_disp_Xoffset
        l2=0.0
        hand_z_off=0.46
        l3 = hand_z_off  
        tip_hand_transform = [l1, l2, l3, 0,0,0] # to be updated when we have a hand design finalized
        # broadcast frame attached to tcp
        pubFrame(br, pose=tip_hand_transform, frame_id='tip', parent_frame_id='link_6', npub=5)
        
        #~ *************************************************************
        # MOVE TO MOUTH OF THE BIN
        q_initial = robotConfig
        # move to bin mouth
        if steeper_grasp==True:
            distFromShelf_outbin = 0.25
        else:
            distFromShelf_outbin=0.1
            
        mouthPt,temp = getBinMouthAndFloor(distFromShelf_outbin, binNum)
        mouthPt = coordinateFrameTransform(mouthPt, 'shelf', 'map', listener)
        targetPosition = [mouthPt.pose.position.x, mouthPt.pose.position.y, mouthPt.pose.position.z]
        gripperOri=[0, 0.7071, 0, 0.7071]
           
        
        pubFrame(br, pose=tip_hand_transform, frame_id='target_pose', parent_frame_id='map', npub=10)

        planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = gripperOri, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        plan.setSpeedByName('fastest')
        s = plan.success()
        if s:
            print '[Grasp] move to bin mouth in grasp code successful'
            #plan.visualize()
            plans.append(plan) # Plan 0
            #if isExecute:
            #    pauseFunc(withPause)
             #   plan.execute()
        else:
            print '[Grasp] move to bin mouth in grasp code fail'
            # drake_success=False
            # grasp_possible=False
            return (False, False)
        
        qf = plan.q_traj[-1]
        
        q_initial = qf
        
        #~ *************************************************************
        
        ###### MOVE THE ROBOT OUTSIDE THE BIN IN GRASP ORIENTATION ######

        #~ Move the robot outside the bin but with grasp orientation 
        # set grasp_tcp
        grasp_l1=hand_fing_disp_Xoffset #0.0183
        grasp_l2=0.0
        grasp_l3 = hand_z_off-tcp_edge_off
        grasp_tip_hand_transform = [grasp_l1, grasp_l2, grasp_l3, 0,0,0] # to be updated when we have a hand design finalized
        # broadcast frame attached to grasp_tcp
        
        cheezit_hand_transform=deepcopy(grasp_tip_hand_transform)
        
        pubFrame(br, pose=grasp_tip_hand_transform, frame_id='gtip', parent_frame_id='link_6', npub=5)
                
        #~ q_initial = robotConfig
        # move to bin mouth with grasp orientation
        if steeper_grasp==True:
            distFromShelf_outbin_grasp = 0.2
        else:
            distFromShelf_outbin_grasp=0.1
            
        mouthPt,temp = getBinMouthAndFloor(distFromShelf_outbin_grasp, binNum)
        mouthPt = coordinateFrameTransform(mouthPt, 'shelf', 'map', listener)
        #~ pregrasp_targetPosition = [mouthPt.pose.position.x, graspPT_pose_pos[1], mouthPt.pose.position.z]
        
        offset_in_hand_Z=(graspDeep_pos[0]-mouthPt.pose.position.x)/Cos_angle_made_with_shelf_Y
        offest_in_grasp_frame=[0, 0, offset_in_hand_Z]
        offest_in_grasp_frame=np.array(offest_in_grasp_frame)
        
        offset_in_world=np.dot(graspPT_pose_orient_Rotmat,offest_in_grasp_frame.transpose())
        
        pregrasp_targetPosition=graspDeep_pos-offset_in_world
        pregrasp_targetPosition[2]=pregrasp_targetPosition[2]+hand_lift
        
        #~ print '[Grasp] pregrasp_targetPosition'
        #~ print pregrasp_targetPosition
        
        #~ br = tf.TransformBroadcaster()
        #~ rospy.sleep(0.1)
        
        # rospy.sleep(0.1)
        # for j in range(10):
            # rospy.sleep(0.1)
            # br.sendTransform(tuple(pregrasp_targetPosition), tuple(graspDeep_orient), rospy.Time.now(), 'target_pose', "map")
        # rospy.sleep(0.1)
        pubFrame(br, pose=list(pregrasp_targetPosition)+list(graspDeep_orient), frame_id='target_pose', parent_frame_id='map', npub=5)
        
        planner = IK(q0 = q_initial, target_tip_pos = pregrasp_targetPosition, target_tip_ori = graspDeep_orient, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        plan.setSpeedByName('fastest')
        s = plan.success()
        if s:
            print '[Grasp] move to bin mouth in grasp orient successful'
            #~ print '[Grasp] tcp at:'
            #~ print(pregrasp_targetPosition)
            #~ plan.visualize()
            plans.append(plan) # Plan 1
            #~ if isExecute:
                #~ pauseFunc(withPause)
                #~ plan.execute()
        else:
            print '[Grasp] move to bin mouth in grasp orient fail'
            # drake_success=False
            # grasp_possible=False
            return (False, False)
        
        qf = plan.q_traj[-1]
    
        q_initial = qf
        
        #~ *************************************************************
        
        ############## OPEN THE GRIPPER PREGRASP ###############
        
        # close the hand slighlty lesser than the required grasp width
        pregrasp_opening=grasp_width_pregrasp
        if pregrasp_opening>grasp_range_lim:
            pregrasp_opening=grasp_range_lim
        
        print '[Grasp] hand moving to', pregrasp_opening*1000.0
        grasp_plan = EvalPlan('graspGripper(%f, 100)' % (pregrasp_opening))
        plans.append(grasp_plan)
        #~ graspGripper(pregrasp_opening,100)
        
        #~ graspGripper(0.1,100)
        
        #~ *************************************************************
        #rospy.sleep(1)
        
        ############  MOVE THE ROBOT TO THE GRASP POSE(ELEVATED) ##############
        
        # move to the elevated grasp pose
        #~ pdb.set_trace()
        graspDeep_highpos=deepcopy(graspDeep_pos)
        cheezit_grasp_pos=deepcopy(graspDeep_pos)
        graspDeep_highpos[2]=graspDeep_highpos[2]+hand_lift
        
        # rospy.sleep(0.1)
        # for j in range(10):
            # rospy.sleep(0.1)
            # br.sendTransform(tuple(graspDeep_highpos), tuple(graspDeep_orient), rospy.Time.now(), 'target_pose', "map")
        # rospy.sleep(0.1)
        
        pubFrame(br, pose=list(graspDeep_highpos)+list(graspDeep_orient), frame_id='target_pose', parent_frame_id='link_6', npub=5)
        
        #~ pdb.set_trace()
        # planner = IK(q0 = q_initial, target_tip_pos = graspPT_pose_pos, target_tip_ori = graspPT_pose_orient, 
             # joint_topic=joint_topic, tip_hand_transform=tip_hand_transform, straightness = 0.3, inframebb = inframebb)
        planner = IK(q0 = q_initial, target_tip_pos = graspDeep_highpos, target_tip_ori = graspDeep_orient, 
             joint_topic=joint_topic, tip_hand_transform=grasp_tip_hand_transform)
        plan = planner.plan()
        plan.setSpeedByName('faster')
        s=plan.success()
        if s:
            print '[Grasp] move to higher grasp pose successful'
            # print '[Grasp] tcp at:'
            # print(graspDeep_highpos)
            #~ plan.visualize()
            plans.append(plan) # Plan 2
            #~ if isExecute:
                #~ pauseFunc(withPause)
                #~ plan.execute()
        else:
            print '[Grasp] move to higher grasp pose fail'
            # drake_success=False
            return (False, False)
            
        qf = plan.q_traj[-1]
        
        q_initial = qf
        #~ *************************************************************
         ############  MOVE THE ROBOT TO THE GRASP POSE ##############
        
        # move to the elevated grasp pose
        # rospy.sleep(0.1)
        # for j in range(10):
            # rospy.sleep(0.1)
            # br.sendTransform(tuple(graspDeep_pos), tuple(graspDeep_orient), rospy.Time.now(), 'target_pose', "map")
        # rospy.sleep(0.1)
        
        pubFrame(br, pose=list(graspDeep_pos)+list(graspDeep_orient), frame_id='target_pose', parent_frame_id='map', npub=5)
        
        planner = IK(q0 = q_initial, target_tip_pos = graspDeep_pos, target_tip_ori = graspDeep_orient, 
             joint_topic=joint_topic, tip_hand_transform=grasp_tip_hand_transform)
        plan = planner.plan()
        plan.setSpeedByName('faster')
        s=plan.success()
        if s:
            print '[Grasp] move to grasp pose successful'
            # print '[Grasp] tcp at:'
            # print(graspDeep_pos)
            #~ plan.visualize()
            plans.append(plan) # Plan 3
            #~ if isExecute:
                #~ pauseFunc(withPause)
                #~ plan.execute()
        else:
            print '[Grasp] move to grasp pose fail'
            # drake_success=False
            return (False, False)
            
        qf = plan.q_traj[-1]
        q_initial = qf
        
        q_cheezit=deepcopy(q_initial)
        
        #rospy.sleep(2)
        
        #################### CLOSE THE GRIPPER GRASP #################
        
        # grasp the object (close the hand)
        #~ grasp_closing=grasp_width-0.005
        #~ if grasp_closing<0.005:
            #~ grasp_closing=grasp_width
        #~ 
        #~ print '[Grasp] hand moving to '
        #~ print grasp_closing*1000.0
        #~ graspGripper(grasp_closing,50)
        
        grasp_plan = EvalPlan('graspGripper(%f, 100)' % (0.0))
        plans.append(grasp_plan)
        
        #~ graspGripper(0.01,50)
        #~ *************************************************************
        #rospy.sleep(2)
        
        graspGripper(0.0,100)
        
        # #################### EXECUTE FORWARD #################### 
        if cheezit_grasp==False:
            for numOfPlan in range(0, len(plans)):
                if isExecute:
                    plans[numOfPlan].visualize()
                    pauseFunc(withPause)
                    plans[numOfPlan].execute()
        
        # #################### RETREAT #################### 
        #*************** Execute this only if cheezit grasp is False*****************
        #Ohterwise execute our special cheez it retract********************
        
            for numOfPlan in range(0, len(plans)):
                if isExecute:
                    plans[len(plans)-numOfPlan-1].visualizeBackward()
                    pauseFunc(withPause)
                    plans[len(plans)-numOfPlan-1].executeBackward()
            
    #*******************************************************************
    
    #~ ****************************************************************
    left_flush=False
    right_flush=False
    flush_grasp_possible=False

    flush_grasp_clearance=0.015
    side_wall_clearance=0.010
    left_side_bin=[0,3,6,9]
    right_side_bin=[2,5,8,11]
    hand_rot_X_angle=deepcopy(grasp_tilt_angle)
    hand_rot_Y_angle=5.0
    
    obj_off=0.005
    
    if try_flush_grasp:
        flush_hand_lift=0.035
        bin_mid_PT=bin_mid_pos_world.pose.position.y
        obj_pos_Y=obj_pose_pos[1]
        
        if obj_pos_Y>bin_mid_PT:
            print '[Grasp] will try left wall flush grasp' 
            left_flush=True
        else:
            right_flush=True
            print '[Grasp] will try right wall flush grasp' 
    
    flushGrasp_objlist = ['kong_duck_dog_toy', 'kong_sitting_frog_dog_toy', 'kyjen_squeakin_eggs_plush_puppies','kong_air_dog_squeakair_tennis_ball','dr_browns_bottle_brush']
    
    Inbin_off=0.044
    tiny_obj_list=['kong_duck_dog_toy', 'kong_sitting_frog_dog_toy','champion_copper_plus_spark_plug']
    
    
    if objId in tiny_obj_list:
        flush_X_off=0.040 #0.035
    else:
        flush_X_off=0.095  #0.065
    
    if objId=='elmers_washable_no_run_school_glue':
        flush_X_off=0.115
        
    
    backWall=bin_inner_cnstr[binNum][2]
        
    backWall_world = coordinateFrameTransform([0,backWall,0], 'shelf', 'map', listener)
        
        
    
    if left_flush:
        
        partial_Z_angle=-45.0
        full_Z_angle=-75.0
        
        move_closer=0.0
        if binNum in left_side_bin:
            hand_rot_Y_angle=18.0
            move_closer=0.019
            Inbin_off=0.045
            
        flush_hand_pos_X=graspDeep_pos[0]+flush_X_off
        if flush_hand_pos_X>backWall_world.pose.position.x-0.010:
            flush_hand_pos_X=backWall_world.pose.position.x-0.010
            
        
        flush_hand_pos_Y=binLeftWall_world.pose.position.y - gripper_width/2.0 + hand_fing_disp_Xoffset + move_closer #obj_pos_Y-flush_grasp_clearance
        
        obj_right_edge=obj_pose_pos[1]-diag_dist/2.0-obj_off
        
        if (flush_hand_pos_Y-grasp_range_lim/2.0<obj_right_edge) or objId in flushGrasp_objlist:
            flush_grasp_possible=True
        else:
            print '[Grasp] The object is not close enough to the left wall to be grasped'
            flush_grasp_possible=False
        
        #~ max_posbl_Y=binLeftWall_world.pose.position.y - hand_width/2.0 - side_wall_clearance
        #~ if flush_hand_pos_Y>max_posbl_Y:
            #~ flush_hand_pos_Y>max_posbl_Y
            
        flush_hand_pos_Z=graspDeep_pos[2]+0.003
        flush_hand_pos=np.array([flush_hand_pos_X,flush_hand_pos_Y,flush_hand_pos_Z])
        
        
        Cos_angle_made_with_shelf_Y=1.0
        hand_norm_vec=np.array([1,0,0])
        fing_axis_vec=np.array([0,1,0])
        hand_Y=np.array([0,0,1])
        flush_graspPT_pose_orient_wo_tilt=np.vstack((fing_axis_vec,hand_Y,hand_norm_vec)) # We arange hand normal, fing axis such that they match Z and X axis of the hand resp.
        flush_graspPT_pose_orient_wo_tilt=graspPT_pose_orient_wo_tilt.transpose()
        
        flush_graspPT_pose_orient=np.dot(graspPT_pose_orient_wo_tilt,rotmatX(hand_rot_X_angle))
        
        cheezit_graspPT_pose_orient_NOY=deepcopy(flush_graspPT_pose_orient)
        
        flush_graspPT_pose_orient=np.dot(flush_graspPT_pose_orient,rotmatY(hand_rot_Y_angle))
        
        flush_graspPT_pose_orient_Rotmat=deepcopy(flush_graspPT_pose_orient)
        
        flush_graspPT_pose_orient = [[flush_graspPT_pose_orient[0][0],flush_graspPT_pose_orient[0][1],flush_graspPT_pose_orient[0][2],0],
                               [flush_graspPT_pose_orient[1][0],flush_graspPT_pose_orient[1][1],flush_graspPT_pose_orient[1][2],0],
                               [flush_graspPT_pose_orient[2][0],flush_graspPT_pose_orient[2][1],flush_graspPT_pose_orient[2][2],0],
                               [0,0,0,1]]

        flush_graspPT_pose_orient = np.array(flush_graspPT_pose_orient)
        
        flush_graspPT_pose_orient_quat=quat_from_matrix(flush_graspPT_pose_orient)
        cheezit_graspPT_pose_orient_quat=deepcopy(flush_graspPT_pose_orient_quat)
    
    if right_flush:
        
        partial_Z_angle=45.0
        full_Z_angle=75.0
        
        move_closer=0.0
        if binNum in right_side_bin:
            hand_rot_Y_angle=18.0
            move_closer=0.019
            Inbin_off=0.045
            
        # print '[Grasp] diag_dist=',diag_dist
        # 
        # if diag_dist < 0.080:
            # flush_hand_pos_X=graspDeep_pos[0] + 0.080
        # elif diag_dist > 0.120:
            # flush_hand_pos_X=graspDeep_pos[0] + 0.120
        # else:
            # flush_hand_pos_X=graspDeep_pos[0]+diag_dist
            
        flush_hand_pos_X=graspDeep_pos[0]+flush_X_off
        
        
        if flush_hand_pos_X>backWall_world.pose.position.x-0.010:
            flush_hand_pos_X=backWall_world.pose.position.x-0.010
        
        
        flush_hand_pos_Y=binRightWall_world.pose.position.y + gripper_width/2.0 - hand_fing_disp_Xoffset + side_wall_clearance -move_closer#obj_pos_Y+flush_grasp_clearance
        
        obj_left_edge=obj_pose_pos[1]+diag_dist/2.0+obj_off
        
        if ((flush_hand_pos_Y+grasp_range_lim/2.0)>obj_left_edge) or objId in flushGrasp_objlist:
            flush_grasp_possible=True
        else:
            print '[Grasp] The object is not close enough to the right wall to be grasped'
            flush_grasp_possible=False
            
        
        #~ max_posbl_Y=binLeftWall_world.pose.position.y - hand_width/2.0 - side_wall_clearance
        #~ if flush_hand_pos_Y>max_posbl_Y:
            #~ flush_hand_pos_Y>max_posbl_Y
        
        flush_hand_pos_Z=graspDeep_pos[2]+0.003
        
        flush_hand_pos=np.array([flush_hand_pos_X,flush_hand_pos_Y,flush_hand_pos_Z])
        
        # print '[Grasp] flush_hand_pos=' ,flush_hand_pos
        # hand_rot_Y_angle=0.0
        Cos_angle_made_with_shelf_Y=1.0
        hand_norm_vec=np.array([1,0,0])
        fing_axis_vec=np.array([0,-1.0,0])
        hand_Y=np.array([0,0,-1.0])
        flush_graspPT_pose_orient_wo_tilt=np.vstack((fing_axis_vec,hand_Y,hand_norm_vec)) # We arange hand normal, fing axis such that they match Z and X axis of the hand resp.
        
        # print '[Grasp] flush_graspPT_pose_orient_wo_tilt=',flush_graspPT_pose_orient_wo_tilt
        
        flush_graspPT_pose_orient_wo_tilt=flush_graspPT_pose_orient_wo_tilt.conj().transpose()
        # flushPT_pose_tfm_list=matrix_from_xyzquat([0,0,0],[0.5,-0.5,0.5,-0.5])
        # flushPT_pose_tfm=np.array(flushPT_pose_tfm_list)
# 
        # flush_graspPT_pose_orient_wo_tilt=flushPT_pose_tfm[0:3,0:3]
        # print '[Grasp] flush_graspPT_pose_orient_wo_tilt=',flush_graspPT_pose_orient_wo_tilt
        # print '[Grasp] flushPT_pose_orient=',flushPT_pose_orient
        
        # print '[Grasp] rotmatY(hand_rot_Y_angle)=',rotmatY(hand_rot_Y_angle)
        # print '[Grasp] rotmatX(hand_rot_X_angle)=',rotmatX(hand_rot_X_angle)
        
        # flush_graspPT_pose_orient=np.dot(flush_graspPT_pose_orient_wo_tilt,np.identity(3))
        flush_graspPT_pose_orient=np.dot(flush_graspPT_pose_orient_wo_tilt,rotmatX(-hand_rot_X_angle))
        # print '[Grasp] flush_graspPT_pose_orient1=',flush_graspPT_pose_orient
        
        cheezit_graspPT_pose_orient_NOY=deepcopy(flush_graspPT_pose_orient)
        
        flush_graspPT_pose_orient=np.dot(flush_graspPT_pose_orient,rotmatY(hand_rot_Y_angle))
        # print '[Grasp] flush_graspPT_pose_orient2=',flush_graspPT_pose_orient
        
        flush_graspPT_pose_orient_Rotmat=deepcopy(flush_graspPT_pose_orient)
        
        flush_graspPT_pose_orient = [[flush_graspPT_pose_orient[0][0],flush_graspPT_pose_orient[0][1],flush_graspPT_pose_orient[0][2],0],
                               [flush_graspPT_pose_orient[1][0],flush_graspPT_pose_orient[1][1],flush_graspPT_pose_orient[1][2],0],
                               [flush_graspPT_pose_orient[2][0],flush_graspPT_pose_orient[2][1],flush_graspPT_pose_orient[2][2],0],
                               [0,0,0,1]]
        
        # print '[Grasp] flush_graspPT_pose_orient3=',flush_graspPT_pose_orient
        
        flush_graspPT_pose_orient = np.array(flush_graspPT_pose_orient)
        
        # print '[Grasp] flush_graspPT_pose_orient4=',flush_graspPT_pose_orient
        
        flush_graspPT_pose_orient_quat=quat_from_matrix(flush_graspPT_pose_orient)
        cheezit_graspPT_pose_orient_quat=deepcopy(flush_graspPT_pose_orient_quat)
        
        # print '[Grasp] flush_graspPT_pose_orient_quat=',flush_graspPT_pose_orient_quat
        
    if objId =='cheezit_big_original' and obj_upright and flush_grasp_possible:
        print '[Grasp] Cheezit is in flush graspable pose but upright, will execute special primitive after flush grasp'
        #~ grasp_possible=False
        cheezit_grasp=True
        cheezit_with_flushgrasp=True
        
    if flush_grasp_possible:
        #Let's do robot motion planning now
        # plan store
        drake_success=True
        flush_plans = []
        
        # set tcp (same as that set in generate dictionary)
        l1=hand_fing_disp_Xoffset
        l2=0.0
        hand_z_off=0.46
        l3 = hand_z_off  
        tip_hand_transform = [l1, l2, l3, 0,0,0] # to be updated when we have a hand design finalized
        cheezit_hand_transform=deepcopy(tip_hand_transform)
        
        # broadcast frame attached to tcp
        pubFrame(br, pose=tip_hand_transform, frame_id='tip', parent_frame_id='link_6', npub=5)
        
        #~ *************************************************************
        # MOVE TO MOUTH OF THE BIN
        q_initial = robotConfig
        # move to bin mouth
        if steeper_grasp==True:
            distFromShelf_outbin = 0.25
        else:
            distFromShelf_outbin=0.1
            
        mouthPt,temp = getBinMouthAndFloor(distFromShelf_outbin, binNum)
        mouthPt = coordinateFrameTransform(mouthPt, 'shelf', 'map', listener)
        targetPosition = [mouthPt.pose.position.x, mouthPt.pose.position.y, mouthPt.pose.position.z]
        gripperOri=[0, 0.7071, 0, 0.7071]
        
        pubFrame(br, pose=tip_hand_transform, frame_id='target_pose', parent_frame_id='map', npub=10)

        planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = gripperOri, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        plan.setSpeedByName('fastest')
        s = plan.success()
        if s:
            print '[Grasp] move to bin mouth in flush grasp part successful'
            #plan.visualize()
            flush_plans.append(plan) # Plan 0
            #if isExecute:
            #    pauseFunc(withPause)
             #   plan.execute()
        else:
            print '[Grasp] move to bin mouth in flush grasp part fail'
            # drake_success= False
            # flush_grasp_possible=False
            return (False, False)
        
        qf = plan.q_traj[-1]
        
        q_initial = qf
        
        #################### CLOSE THE GRIPPER GRASP #################
        
        grasp_plan = EvalPlan('graspGripper(%f, 100)' % (0.0))
        flush_plans.append(grasp_plan)
        
        #~ *************************************************************
        
        ###### MOVE THE ROBOT JUST INSIDE THE BIN IN GRASP ORIENTATION ######

        #~ Move the robot outside the bin but with grasp orientation 
        # set grasp_tcp
        grasp_l1=hand_fing_disp_Xoffset #0.0183
        grasp_l2=0.0
        grasp_l3 = hand_z_off
        grasp_tip_hand_transform = [grasp_l1, grasp_l2, grasp_l3, 0,0,0] # to be updated when we have a hand design finalized
        # broadcast frame attached to grasp_tcp
        # rospy.sleep(0.1)
        # for j in range(5):
            # rospy.sleep(0.1)
            # br.sendTransform(tuple(grasp_tip_hand_transform[0:3]), tfm.quaternion_from_euler(*grasp_tip_hand_transform[3:6]), rospy.Time.now(), 'gtip', "link_6")
        # rospy.sleep(0.1)
        
        pubFrame(br, pose=grasp_tip_hand_transform, frame_id='gtip', parent_frame_id='link_6', npub=5)
                
        #~ q_initial = robotConfig
        # move to bin mouth with grasp orientation
         
        pregrasp_targetPosition=deepcopy(flush_hand_pos)
        pregrasp_targetPosition[0]=mouthPt.pose.position.x
        pregrasp_targetPosition[2]=pregrasp_targetPosition[2]+flush_hand_lift
        
        #~ print '[Grasp] pregrasp_targetPosition'
        #~ print pregrasp_targetPosition
        
        #~ br = tf.TransformBroadcaster()
        #~ rospy.sleep(0.1)
        
        # rospy.sleep(0.1)
        # for j in range(10):
            # rospy.sleep(0.1)
            # br.sendTransform(tuple(pregrasp_targetPosition), tuple(flush_graspPT_pose_orient_quat), rospy.Time.now(), 'target_pose', "map")
        # rospy.sleep(0.1)
        
        pubFrame(br, pose=list(pregrasp_targetPosition)+list(flush_graspPT_pose_orient_quat), frame_id='target_pose', parent_frame_id='map', npub=5)
        
        planner = IK(q0 = q_initial, target_tip_pos = pregrasp_targetPosition, target_tip_ori = flush_graspPT_pose_orient_quat, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        plan.setSpeedByName('fastest')
        s = plan.success()
        if s:
            print '[Grasp] move to  bin mouth in grasp orient successful'
            #~ print '[Grasp] tcp at:'
            #~ print(pregrasp_targetPosition)
            #~ plan.visualize()
            flush_plans.append(plan) # Plan 1
            #~ if isExecute:
                #~ pauseFunc(withPause)
                #~ plan.execute()
        else:
            print '[Grasp] move to bin mouth in grasp orient fail'
            # drake_success= False
            # flush_grasp_possible=False
            return (False, False)
            
        qf = plan.q_traj[-1]
    
        q_initial = qf
        
        
        # MOVE SLIGHTLY IN
        distFromShelf_outbin=-Inbin_off
        # distFromShelf_outbin=-distFromShelf_outbin
        
        InmouthPt,temp = getBinMouthAndFloor(distFromShelf_outbin, binNum)
        InmouthPt = coordinateFrameTransform(InmouthPt, 'shelf', 'map', listener)
        targetPosition = [InmouthPt.pose.position.x, flush_hand_pos[1], flush_hand_pos[2]+flush_hand_lift]
        
        pubFrame(br, pose=tip_hand_transform, frame_id='target_pose', parent_frame_id='map', npub=10)

        planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = flush_graspPT_pose_orient_quat, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        plan.setSpeedByName('faster')
        s = plan.success()
        if s:
            print '[Grasp] move to inside bin mouth in flush grasp part successful'
            #plan.visualize()
            flush_plans.append(plan) # Plan 0
            #if isExecute:
            #    pauseFunc(withPause)
             #   plan.execute()
        else:
            print '[Grasp] move to inside bin mouth in flush grasp part fail'
            # drake_success= False
            # flush_grasp_possible=False
            return (False, False)
            
        qf = plan.q_traj[-1]
        
        q_initial = qf
        
        #~ *************************************************************
        
        ############## OPEN THE GRIPPER WITH FORCE CONTROL ###############
        
        # close the hand slighlty lesser than the required grasp width
        pregrasp_opening=grasp_width_pregrasp
        if pregrasp_opening>grasp_range_lim:
            pregrasp_opening=grasp_range_lim
        
        print '[Grasp] hand moving to' , (0.110*1000.0)
        grasp_plan = EvalPlan('setForceGripper(%f)' % (10))
        flush_plans.append(grasp_plan)
        #rospy.sleep(1)
        grasp_plan = EvalPlan('graspGripper(%f, 100)' % (0.110))
        flush_plans.append(grasp_plan)
        
        ############  MOVE THE ROBOT TO THE HIGHER GRASP POSE ##############
        # 
        # move to the elevated grasp pose
        # #~ pdb.set_trace()
        # graspDeep_highpos=deepcopy(flush_hand_pos)
        # graspDeep_highpos[2]=graspDeep_highpos[2]+flush_hand_lift
        # graspDeep_highpos[0]=graspDeep_highpos[0]+0.050
        # 
        # rospy.sleep(0.1)
        # for j in range(10):
            # rospy.sleep(0.1)
            # br.sendTransform(tuple(graspDeep_highpos), tuple(flush_graspPT_pose_orient_quat), rospy.Time.now(), 'target_pose', "map")
        # rospy.sleep(0.1)
        # 
        # #~ pdb.set_trace()
        # planner = IK(q0 = q_initial, target_tip_pos = graspPT_pose_pos, target_tip_ori = graspPT_pose_orient, 
             # joint_topic=joint_topic, tip_hand_transform=tip_hand_transform, straightness = 0.3, inframebb = inframebb)
        # planner = IK(q0 = q_initial, target_tip_pos = graspDeep_highpos, target_tip_ori = flush_graspPT_pose_orient_quat, 
             # joint_topic=joint_topic, tip_hand_transform=grasp_tip_hand_transform)
        # plan = planner.plan()
        # plan.setSpeedByName('fast')
        # s=plan.success()
        # if s:
            # print '[Grasp] move to higher grasp pose successful'
            # print '[Grasp] tcp at:'
            # print(graspDeep_highpos)
            # #~ plan.visualize()
            # flush_plans.append(plan) # Plan 2
            # #~ if isExecute:
                # #~ pauseFunc(withPause)
                # #~ plan.execute()
        # else:
            # print '[Grasp] move to higher grasp pose fail'
            # return False
        # qf = plan.q_traj[-1]
        # 
        # q_initial = qf
        #~ *************************************************************
         ############  MOVE THE ROBOT TO THE GRASP POSE ##############
        
        flushgrasp_pos=deepcopy(flush_hand_pos)
        cheezit_grasp_pos=deepcopy(flush_hand_pos)
        
        # rospy.sleep(0.1)
        # for j in range(10):
            # rospy.sleep(0.1)
            # br.sendTransform(tuple(flushgrasp_pos), tuple(flush_graspPT_pose_orient_quat), rospy.Time.now(), 'target_pose', "map")
        # rospy.sleep(0.1)
        
        pubFrame(br, pose=list(flushgrasp_pos)+list(flush_graspPT_pose_orient_quat), frame_id='target_pose', parent_frame_id='map', npub=5)
        
        planner = IK(q0 = q_initial, target_tip_pos = flushgrasp_pos, target_tip_ori = flush_graspPT_pose_orient_quat, 
             joint_topic=joint_topic, tip_hand_transform=grasp_tip_hand_transform)
        plan = planner.plan()
        plan.setSpeedByName('faster')
        s=plan.success()
        if s:
            print '[Grasp] move to grasp pose successful'
            # print '[Grasp] tcp at:'
            # print(flushgrasp_pos)
            #~ plan.visualize()
            flush_plans.append(plan) # Plan 3
            #~ if isExecute:
                #~ pauseFunc(withPause)
                #~ plan.execute()
        else:
            print '[Grasp] move to grasp pose fail'
            # drake_success= False
            # flush_grasp_possible=False
            return (False, False)
            
        qf = plan.q_traj[-1]
        q_initial = qf
        
        q_cheezit=deepcopy(q_initial)
        
        #rospy.sleep(2)
        
        #################### CLOSE THE GRIPPER GRASP #################
        grasp_plan = EvalPlan('setForceGripper(%f)' % (50.0))
        flush_plans.append(grasp_plan)
        #rospy.sleep(1)       
        grasp_plan = EvalPlan('graspGripper(%f, 100)' % (0.0))
        flush_plans.append(grasp_plan)
        
        #~ *************************************************************
        #rospy.sleep(2)
        
        # #################### EXECUTE FORWARD #################### 
        # ***** Execute this only if cheezit grasp it False ***********
        if cheezit_grasp==False:
            for numOfPlan in range(0, len(flush_plans)):
                if isExecute:
                    flush_plans[numOfPlan].visualize()
                    pauseFunc(withPause)
                    flush_plans[numOfPlan].execute()
            
        # #################### RETREAT #################### 
            
            for numOfPlan in range(0, len(flush_plans)):
                if isExecute:
                    flush_plans[len(flush_plans)-numOfPlan-1].visualizeBackward()
                    pauseFunc(withPause)
                    flush_plans[len(flush_plans)-numOfPlan-1].executeBackward()
                
    #*******************************************************************
    if cheezit_grasp==True:
        print '[Grasp] Computing special retreat for cheezit'
        
        #***********************************************************************
        cheezit_graspPT_pose_orient_NOY_mat=deepcopy(cheezit_graspPT_pose_orient_NOY)
        
        cheezit_graspPT_pose_orient_partialZ=np.dot(cheezit_graspPT_pose_orient_NOY_mat,rotmatZ(partial_Z_angle))
        
        cheezit_graspPT_pose_orient_fullZ=np.dot(cheezit_graspPT_pose_orient_NOY_mat,rotmatZ(full_Z_angle))
        
        #****************************************
        cheezit_graspPT_pose_orient_NOY = [[cheezit_graspPT_pose_orient_NOY[0][0],cheezit_graspPT_pose_orient_NOY[0][1],cheezit_graspPT_pose_orient_NOY[0][2],0],
                               [cheezit_graspPT_pose_orient_NOY[1][0],cheezit_graspPT_pose_orient_NOY[1][1],cheezit_graspPT_pose_orient_NOY[1][2],0],
                               [cheezit_graspPT_pose_orient_NOY[2][0],cheezit_graspPT_pose_orient_NOY[2][1],cheezit_graspPT_pose_orient_NOY[2][2],0],
                               [0,0,0,1]]

        cheezit_graspPT_pose_orient_NOY = np.array(cheezit_graspPT_pose_orient_NOY)
        
        cheezit_graspPT_pose_orient_NOY_quat=quat_from_matrix(cheezit_graspPT_pose_orient_NOY)
        
        #***************************************88
        
        cheezit_graspPT_pose_orient_partialZ = [[cheezit_graspPT_pose_orient_partialZ[0][0],cheezit_graspPT_pose_orient_partialZ[0][1],cheezit_graspPT_pose_orient_partialZ[0][2],0],
                               [cheezit_graspPT_pose_orient_partialZ[1][0],cheezit_graspPT_pose_orient_partialZ[1][1],cheezit_graspPT_pose_orient_partialZ[1][2],0],
                               [cheezit_graspPT_pose_orient_partialZ[2][0],cheezit_graspPT_pose_orient_partialZ[2][1],cheezit_graspPT_pose_orient_partialZ[2][2],0],
                               [0,0,0,1]]

        cheezit_graspPT_pose_orient_partialZ = np.array(cheezit_graspPT_pose_orient_partialZ)
        
        cheezit_graspPT_pose_orient_partialZ_quat=quat_from_matrix(cheezit_graspPT_pose_orient_partialZ)
        
        #******************************************
        
        cheezit_graspPT_pose_orient_fullZ = [[cheezit_graspPT_pose_orient_fullZ[0][0],cheezit_graspPT_pose_orient_fullZ[0][1],cheezit_graspPT_pose_orient_fullZ[0][2],0],
                               [cheezit_graspPT_pose_orient_fullZ[1][0],cheezit_graspPT_pose_orient_fullZ[1][1],cheezit_graspPT_pose_orient_fullZ[1][2],0],
                               [cheezit_graspPT_pose_orient_fullZ[2][0],cheezit_graspPT_pose_orient_fullZ[2][1],cheezit_graspPT_pose_orient_fullZ[2][2],0],
                               [0,0,0,1]]

        cheezit_graspPT_pose_orient_fullZ = np.array(cheezit_graspPT_pose_orient_fullZ)
        
        cheezit_graspPT_pose_orient_fullZ_quat=quat_from_matrix(cheezit_graspPT_pose_orient_fullZ)
        #*******************************************
        
        #**** USE SAME HAND TRANSFORM AS THE GRASP PRIMITIVE EITHER GRASP OR FLUSH GRASP *****
        
        cheezit_lift=0.005
        cheezit_lift_pos=deepcopy(cheezit_grasp_pos)
        cheezit_lift_pos[2]=cheezit_lift_pos[2]+cheezit_lift
        
        cheezit_lift_orient_quat=deepcopy(cheezit_graspPT_pose_orient_quat)
        
        #************* Now we move the hand to center *******************888
        
        
        # MOVE to center
        dist_cheezit_temp=0.0
        # distFromShelf_outbin=-distFromShelf_outbin
        
        if binNum in left_side_bin:
            cheezit_off=-0.015
        elif binNum in right_side_bin:
            cheezit_off=0.015
        else:
            cheezit_off=0.0
            
        
        center_loc,temp = getBinMouthAndFloor(dist_cheezit_temp, binNum)
        center_loc_world = coordinateFrameTransform(center_loc, 'shelf', 'map', listener)
        cheezit_center_pos=deepcopy(cheezit_lift_pos)
        cheezit_center_pos[1]=center_loc_world.pose.position.y+cheezit_off
        
        cheezit_center_orient_quat=deepcopy(cheezit_graspPT_pose_orient_NOY_quat)
        
        #************* Now we tilt the hand *******************888
        #Try to turn it by 75, if drake does not like it we do 45 
        
        cheezit_center_tilt_pos=deepcopy(cheezit_center_pos)
        
        cheezit_center_parttilt_orient_quat=deepcopy(cheezit_graspPT_pose_orient_partialZ_quat)
        cheezit_center_fulltilt_orient_quat=deepcopy(cheezit_graspPT_pose_orient_fullZ_quat)
        
        
        #Now move the hand back
        
        # MOVE the hand back
        dist_cheezit_back=0.250
        # distFromShelf_outbin=-distFromShelf_outbin
        
        back_loc,temp = getBinMouthAndFloor(dist_cheezit_back, binNum)
        back_loc_world = coordinateFrameTransform(back_loc, 'shelf', 'map', listener)
        cheezit_moveback_pos=deepcopy(cheezit_center_tilt_pos)
        cheezit_moveback_pos[0]=back_loc_world.pose.position.x
        
        #~ cheezit_moveback_orient_quat=deepcopy(
        # We have to use the tilt we used in earlier step
        
        # go to default orientation
        cheezit_home_pos=deepcopy(cheezit_moveback_pos)
        cheezit_home_orient_quat=deepcopy(gripperOri)
        
        cheezit_retreat_plans=[]
        
        # Now lets drake the retreat trajectory
        
        #Lifting the hand up *******************
        pubFrame(br, pose=cheezit_hand_transform, frame_id='target_pose', parent_frame_id='map', npub=10)

        planner = IK(q0 = q_cheezit, target_tip_pos = cheezit_lift_pos, target_tip_ori = cheezit_lift_orient_quat, tip_hand_transform=cheezit_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        plan.setSpeedByName('fastest')
        s = plan.success()
        if s:
            print '[Grasp] lift the chezzit box successful'
            #plan.visualize()
            cheezit_retreat_plans.append(plan) # Plan 0
            #if isExecute:
            #    pauseFunc(withPause)
             #   plan.execute()
        else:
            print '[Grasp] lift the chezzit box fail'
            # drake_success= False
            # flush_grasp_possible=False
            return (False, False)
        
        qf = plan.q_traj[-1]
        
        q_initial = qf
        
        # Moving the hand to center w/o tilt*******************
        pubFrame(br, pose=cheezit_hand_transform, frame_id='target_pose', parent_frame_id='map', npub=10)

        planner = IK(q0 = q_initial, target_tip_pos = cheezit_center_pos, target_tip_ori = cheezit_center_orient_quat, tip_hand_transform=cheezit_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        plan.setSpeedByName('fastest')
        s = plan.success()
        if s:
            print '[Grasp] move the chezzit box to center successful'
            #plan.visualize()
            cheezit_retreat_plans.append(plan) # Plan 0
            #if isExecute:
            #    pauseFunc(withPause)
             #   plan.execute()
        else:
            print '[Grasp] Move the chezzit box to center fail'
            # drake_success= False
            # flush_grasp_possible=False
            return (False, False)
        
        qf = plan.q_traj[-1]
        
        q_initial = qf
        
        # Moving the hand to center with tilt*******************
        pubFrame(br, pose=cheezit_hand_transform, frame_id='target_pose', parent_frame_id='map', npub=10)

        planner = IK(q0 = q_initial, target_tip_pos = cheezit_center_tilt_pos, target_tip_ori = cheezit_center_fulltilt_orient_quat, tip_hand_transform=cheezit_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        plan.setSpeedByName('fastest')
        s = plan.success()
        if s:
            print '[Grasp] move the chezzit box to center with full tilt successful'
            #plan.visualize()
            cheezit_retreat_plans.append(plan) # Plan 0
            #if isExecute:
            #    pauseFunc(withPause)
             #   plan.execute()
        else:
            print '[Grasp] Move the chezzit box to center  with full tilt  fail'
            # drake_success= False
            # flush_grasp_possible=False
            try_partial_tilt=True
            #~ return (False, False)
            
        if try_partial_tilt==True:
            pubFrame(br, pose=cheezit_hand_transform, frame_id='target_pose', parent_frame_id='map', npub=10)

            planner = IK(q0 = q_initial, target_tip_pos = cheezit_center_tilt_pos, target_tip_ori = cheezit_center_parttilt_orient_quat, tip_hand_transform=cheezit_hand_transform, joint_topic=joint_topic)
            plan = planner.plan()
            plan.setSpeedByName('fastest')
            s = plan.success()
            if s:
                print '[Grasp] move the chezzit box to center  with partial tilt  successful'
                #plan.visualize()
                cheezit_retreat_plans.append(plan) # Plan 0
                #if isExecute:
                #    pauseFunc(withPause)
                 #   plan.execute()
            else:
                print '[Grasp] Move the chezzit box to center  with partial tilt  fail'
                # drake_success= False
                # flush_grasp_possible=False
                return (False, False)
        
        qf = plan.q_traj[-1]
        
        q_initial = qf
        
        #***Moving the hand back *******************
        pubFrame(br, pose=cheezit_hand_transform, frame_id='target_pose', parent_frame_id='map', npub=10)
        
        if try_partial_tilt==True:
            planner = IK(q0 = q_initial, target_tip_pos = cheezit_moveback_pos, target_tip_ori = cheezit_center_parttilt_orient_quat, tip_hand_transform=cheezit_hand_transform, joint_topic=joint_topic)
        else:
            planner = IK(q0 = q_initial, target_tip_pos = cheezit_moveback_pos, target_tip_ori = cheezit_center_fulltilt_orient_quat, tip_hand_transform=cheezit_hand_transform, joint_topic=joint_topic)
        
        plan = planner.plan()
        plan.setSpeedByName('fastest')
        s = plan.success()
        if s:
            print '[Grasp] move back chezzit box successful'
            #plan.visualize()
            cheezit_retreat_plans.append(plan) # Plan 0
            #if isExecute:
            #    pauseFunc(withPause)
             #   plan.execute()
        else:
            print '[Grasp] move back chezzit box fail'
            # drake_success= False
            # flush_grasp_possible=False
            return (False, False)
        
        qf = plan.q_traj[-1]
        
        q_initial = qf
        
        # Moving in default orientation
        pubFrame(br, pose=cheezit_hand_transform, frame_id='target_pose', parent_frame_id='map', npub=10)

        planner = IK(q0 = q_initial, target_tip_pos = cheezit_home_pos, target_tip_ori = cheezit_home_orient_quat, tip_hand_transform=cheezit_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        plan.setSpeedByName('fastest')
        s = plan.success()
        if s:
            print '[Grasp] moving chezzit box to home  successful'
            #plan.visualize()
            cheezit_retreat_plans.append(plan) # Plan 0
            #if isExecute:
            #    pauseFunc(withPause)
             #   plan.execute()
        else:
            print '[Grasp] moving cheezit box to home fail'
            # drake_success= False
            # flush_grasp_possible=False
            return (False, False)
        
        qf = plan.q_traj[-1]
        
        q_initial = qf
        
        # Run the plans now
        if cheezit_with_flushgrasp:
            for numOfPlan in range(0, len(flush_plans)):
                    if isExecute:
                        flush_plans[numOfPlan].visualize()
                        pauseFunc(withPause)
                        flush_plans[numOfPlan].execute()
        else:
            for numOfPlan in range(0, len(plans)):
                    if isExecute:
                        plans[numOfPlan].visualize()
                        pauseFunc(withPause)
                        plans[numOfPlan].execute()
        
        for numOfPlan in range(0, len(cheezit_retreat_plans)):
                if isExecute:
                    cheezit_retreat_plans[numOfPlan].visualize()
                    pauseFunc(withPause)
                    cheezit_retreat_plans[numOfPlan].execute()
                    
        
    if try_flush_grasp:
        if flush_grasp_possible:
            print '[Grasp] flush_grasp was possible'
            grasp_possible=True
        else:
            print '[Grasp] flush_grasp was not possible'
    
    
    #############Check if grasp was successful or not###################
        
    if flush_grasp_possible or grasp_possible:    
        # get hand joints from published topic  # sensor_msgs/JointState
        while True:
            APCrobotjoints = ROS_Wait_For_Msg(joint_topic, sensor_msgs.msg.JointState).getmsg() 
            q0 = APCrobotjoints.position  
            # since virutal hand will publish with robot joints
            # so there will be 8 
            print '[Grasp] waiting for message'
            if len(q0) == 2 or len(q0) == 8:
                q0 = q0[-2:]    # get last 2
                break
        
        gripper_q0=np.fabs(q0)
        
        print '[Grasp] Gripper width:', gripper_q0
        
        thin_grasp_list=['kong_duck_dog_toy', 'kong_sitting_frog_dog_toy']
        
        if objId in thin_grasp_list:
            drop_thick=0.0000001 #e-7 #may need to tweak
        else:
            drop_thick=0.000001 # e-6
        
        
        if gripper_q0[0] < drop_thick:
            print '[Grasp] ***************'
            print '[Grasp] Could not grasp'
            print '[Grasp] ***************'
            execution_possible = False
        else:
            print '[Grasp] ***************'
            print '[Grasp] Grasp Successful'
            print '[Grasp] ***************'
            execution_possible = True
    #*******************************************************************

    return(grasp_possible,execution_possible)

########################################################################

# To test the function 
if __name__=='__main__':
    
    rospy.init_node('listener', anonymous=True)
    
    #~ posesrv = rospy.ServiceProxy('/pose_service', GetPose)   # should move service name out
    #~ rospy.sleep(0.5)
    #~ data = posesrv('','')
    #~ pos_x= data.pa.object_list[1].pose.position.x
    #~ pos_y= data.pa.object_list[1].pose.position.y
    #~ pos_z= data.pa.object_list[1].pose.position.z
    #~ orient_x= data.pa.object_list[1].pose.orientation.x
    #~ orient_y= data.pa.object_list[1].pose.orientation.y
    #~ orient_z= data.pa.object_list[1].pose.orientation.z
    #~ orient_w= data.pa.object_list[1].pose.orientation.w
  #~ 
    #~ print '[Grasp] obj_pose=',[pos_x,pos_y,pos_z,orient_x,orient_y,orient_z,orient_w] 
    #~ (grasp_possible,execution_possible)=ulti_grasp(objPose=[pos_x,pos_y,pos_z,orient_x,orient_y,orient_z,orient_w],
            #~ binNum=2,
            #~ objId = 'crayola_64_ct',
            #~ bin_contents = None, 
            #~ robotConfig = None,
            #~ grasp_or_not_ang_lim=20,
            #~ grasp_tilt_angle=5.0,
            #~ grasp_range_lim=0.11,
            #~ fing_width=0.06,
            #~ off_perc=0.0,
            #~ isExecute = True,
            #~ withPause = True)

    objPoses = []
    objPoses.append([1.599, 0.450, 1.106, -0.0030, 0.6979, -0.7161, 0.0023]) # upper left corner
    objPoses.append([1.607, -0.494, 1.130, -0.0030, 0.6979, -0.7161, 0.0023]) # uppper right corner
    objPoses.append([1.598, 0.467, 0.412, -0.0030, 0.6911, -0.7227, 0.0023]) # lower left corner
    objPoses.append([1.660, -0.500, 0.379, -0.0384, 0.6970, -0.7150, 0.0368]) #lower right corner
    
    binNums = [0,2,9,11]
    
    for x in range(0,4):
        (grasp_possible,execution_possible)=grasp(objPose = objPoses[x],
            binNum = binNums[x],
            objId = 'crayola_64_ct',
            bin_contents = None, 
            robotConfig = None, 
            grasp_or_not_ang_lim=20,
            grasp_tilt_angle=5.0,
            grasp_range_lim=0.11,
            fing_width=0.06,
            off_perc=0.0,
            isExecute = True,
            withPause = True)
