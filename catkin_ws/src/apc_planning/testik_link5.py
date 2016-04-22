#!/usr/bin/env python

# a minimal example of using python interface for drake, and plan with link5
import geometry_msgs
import std_msgs
import json
from ik.roshelper import ROS_Wait_For_Msg
from ik.helper import xyzrpy_from_xyzquat
from ik.ik import IK
from ik.ik import Plan
from ik.helper import get_bin_cnstr
from ik.helper import get_bin_inner_cnstr
from ik.helper import pause
import tf.transformations as tfm
import rospy
import pdb
import sys
import tf
import math
import numpy as np
import tf.transformations as tfm

import capsen.capsen
from ik.roshelper import pubFrame

# bin_1
# 1.1069; -0.018219; 1.3762 -0.54475; 0.075625; 0.20345; 0.81003
#-0.035814156250925766, 0.5470560007467684, -0.8243085337401062, -2.70526034059, 0.8721846604532582, -0.3148573970598978

rospy.init_node('listener', anonymous=True)
#desired_joint_pose = [-0.0087, 0.6423, -0.1222, 1.4731, 1.4643, 1.2]  # for bin 7
home_joint_pose = [0, -0.2, 0.2, 0.01, 1, 1.4]
plan = Plan(); plan.q_traj = [home_joint_pose]; plan.visualize(); plan.execute()
br = tf.TransformBroadcaster()

# generate targets for horizontal scan
#bin_cnstr = get_bin_cnstr()
bin_cnstr = get_bin_inner_cnstr()


target_poses = []
refind = 7
for i in range(12):
    #target_pose_7_vert = [1.1756, 0, 0.79966, 0.69537, 0.11132, 0.049168, 0.70827]  # for bin 7 vertical
    #target_pose_7_hori = [1.1756, -0.0080496, 0.79966, 0.96262, -0.039399, -0.25934, 0.06743]  # for bin 7 horizontal
    target_pose_7_vert = [1.1756, 0, 0.79966] + tfm.quaternion_from_euler(math.pi, 1.4, math.pi).tolist() # for bin 7 vertical
    target_pose_7_hori = [1.1756, 0, 0.79966] + tfm.quaternion_from_euler(2.2, 0, math.pi/2).tolist()
    target_pose_7_invhori = [1.1756, 0, 0.79966] + tfm.quaternion_from_euler(-math.pi/2, 0, -math.pi/2).tolist()
    
    if i < 3:
        target_pose = target_pose_7_invhori
        if i == 0:
            target_pose[1] -= 0.06
        target_pose[2] -= 0.08  # z
    elif i<6:
        target_pose = target_pose_7_hori
        target_pose[0] += 0.03  # x
        target_pose[2] -= 0.04  # z
    elif i<9:
        target_pose = target_pose_7_hori
    else:
        target_pose = target_pose_7_hori
        target_pose[2] += 0.1  # z

    target_pose[1] += (bin_cnstr[i][0]+bin_cnstr[i][1])/2 - (bin_cnstr[refind][0]+bin_cnstr[refind][1])/2  # y
    target_pose[2] += (bin_cnstr[i][4]+bin_cnstr[i][5])/2 - (bin_cnstr[refind][4]+bin_cnstr[refind][5])/2  # z
    #target_pose[1] += 0.08
    target_poses.append(target_pose)
    
tip_hand_transform = xyzrpy_from_xyzquat([-0.062, 0.021, -0.025, -0.007, 0.704, 0.710, -0.002])


rospy.init_node('listener', anonymous=True)
capsen.capsen.init()

# obj_list = ["champion_copper_plus_spark_plug", "kong_duck_dog_toy", "kong_sitting_frog_dog_toy",
            # "first_years_take_and_toss_straw_cup", "mead_index_cards", "champion_copper_plus_spark_plug",
            # "stanley_66_052", "dr_browns_bottle_brush", "sharpie_accent_tank_style_highlighters",
            # "champion_copper_plus_spark_plug", "sharpie_accent_tank_style_highlighters", "mommys_helper_outlet_plugs"]
obj_list = ["elmers_washable_no_run_school_glue", "paper_mate_12_count_mirado_black_warrior", "feline_greenies_dental_treats",
            "champion_copper_plus_spark_plug", "paper_mate_12_count_mirado_black_warrior", "mark_twain_huckleberry_finn",
            "laugh_out_loud_joke_book", "laugh_out_loud_joke_book", "mark_twain_huckleberry_finn",
            "stanley_66_052", "mead_index_cards", "expo_dry_erase_board_eraser"]
# obj_list = ["champion_copper_plus_spark_plug", "kong_duck_dog_toy", "kong_sitting_frog_dog_toy",
            # "first_years_take_and_toss_straw_cup", "mead_index_cards", "champion_copper_plus_spark_plug",
            # "stanley_66_052", "dr_browns_bottle_brush", "sharpie_accent_tank_style_highlighters",
            # "champion_copper_plus_spark_plug", "sharpie_accent_tank_style_highlighters", "mommys_helper_outlet_plugs"]

# for i in range(0,12):
    
for i in range(0,12):
    pubFrame(br, [0, -0.048, 0, -0.5, 0.5, -0.5, 0.5], 'realsense_depth_optical_frame', 'realsense_link')

    target_pose = target_poses[i]
    print "Trial i=", i, "target_pose=", target_pose
    
    plan = None
    for j in range(100):
        planner = IK(target_tip_pos = target_pose[0:3], target_tip_ori = target_pose[3:7], 
                     tip_hand_transform=tip_hand_transform, 
                     target_link='link_5', target_joint_bb=[[6, 1.2, 1.2]], ori_tol=0.5*j/10.0, pos_tol=0.01*j/10.0)
        newplan = planner.plan()
        if newplan.success():
            plan = newplan
            print 'success'
            break
        else:
            print 'failed'
            
    if plan:
        plan.visualize()
        ## visualize the frame
            
        br.sendTransform(tuple(tip_hand_transform[0:3]), 
                         tfm.quaternion_from_euler(*tip_hand_transform[3:6]), 
                         rospy.Time.now(), 'tip', "link_5")
        br.sendTransform(tuple(target_pose[0:3]), tuple(target_pose[3:7]), 
                         rospy.Time.now(), 'target_pose', "world")

        # user_reply=raw_input('Execute? [(y)es/(n)o]: ')
        # if user_reply == 'y':
            # plan.setSpeedByName('fastest')
        plan.execute()
        raw_input('[NIMA] test realsense now: ')
    
    rospy.sleep(0.8)  # wait for new pointcloud
    obj_pose = capsen.capsen.detectOneObject(obj_list[i], [obj_list[i]], i, mode = 1)
    pause()
    
    # go home
    plan = Plan(); plan.q_traj = [home_joint_pose]; 
    plan.execute()
    
    # if obj_pose is None:
        # print 'Detection failed'
        # continue
        
    # pubFrame(br, obj_pose, 'obj', 'world')

plan = Plan(); plan.q_traj = [home_joint_pose]; 
plan.execute()
    

#rostopic pub /ikTrajServer std_msgs/String "'{\"q0\": [-0.006981317007977334, -0.09599310885968834, -0.024434609527920665, 0.015707963267949, 0.905825881785059, -0.01919862177193767], \"target_hand_ori\": [0.7071, 0, 0.7071, 0], \"target_hand_pos\": [0.94, 0, 1.2965]}'"

# {"q0": [-0.006981317007977334, -0.09599310885968834, -0.024434609527920665, 0.015707963267949, 0.905825881785059, -0.01919862177193767], "target_hand_ori": [0.7071, 0, 0.7071, 0], "target_hand_pos": [0.94, 0, 1.2965]}

# data = JSON.parse('{"q0": [-0.006981317007977334, -0.09599310885968834, -0.024434609527920665, 0.015707963267949, 0.905825881785059, -0.01919862177193767], "target_hand_ori": [1, 0, 0, 0], "target_hand_pose": [1, 0, 1]}')

# rosbag play -l $APCDATA_BASE/bagfiles/2015-04-09-18-26-49.bag --clock --rate 0.1 --topics /robot1_RosJointState

# rosbag play -l $APCDATA_BASE/bagfiles/2015-04-09-18-26-49.bag --topics /kinect2/rgb/image

# roslaunch apc_config robot_perception.launch use_realkinect:=false have_robot:=false

# python $APC_BASE/catkin_ws/src/apc_planning/testik.py

# matlabdrake
# matlabapc

# roslaunch launch mcubeAPCSystem.launch


# 1.7831; 0.0036694; -0.50575
# 0; 0; 0.70711; 0.70711
