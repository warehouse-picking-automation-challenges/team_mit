#!/usr/bin/env python

import os
import goToBin
import goToHome
import goToMouth
import scoop
import push
import topple
import grasp
import sys
import geometry_msgs.msg
import std_msgs
import json
import tf
from ik.roshelper import ROS_Wait_For_Msg
from ik.roshelper import pose2list
from ik.ik import IK
from ik.ik import IKJoint
from ik.helper import find_object_pose_type
from apc.helper import loadHeuristic
import rospy
import pdb
import numpy as np
import math
import tf.transformations as tfm
import gripper
import capsen.capsen
import optparse
import plan_suckdown_func
import suction_down

from ik.helper import xyzrpy_from_xyzquat
from ik.ik import Plan
from ik.helper import get_bin_cnstr
from ik.helper import get_bin_inner_cnstr
from ik.helper import pause
from ik.roshelper import pubFrame

from apc_perception.srv import *
import copy

useKinect = True  # False to never use it, and use hand vision always

objectUseHandVisionList = {
"oreo_mega_stuf": False,
"crayola_64_ct": False,
"paper_mate_12_count_mirado_black_warrior": True,
"mead_index_cards": True,
"rolodex_jumbo_pencil_cup": True,
"mark_twain_huckleberry_finn": True,
"laugh_out_loud_joke_book": True,
"sharpie_accent_tank_style_highlighters": True,
"stanley_66_052": True,
"expo_dry_erase_board_eraser": True,
"champion_copper_plus_spark_plug": True,
"feline_greenies_dental_treats": False,
"kong_air_dog_squeakair_tennis_ball": True,
"dr_browns_bottle_brush": True,
"kong_duck_dog_toy": True,
"kong_sitting_frog_dog_toy": True,
"munchkin_white_hot_duck_bath_toy": True,
"mommys_helper_outlet_plugs": True,
"kyjen_squeakin_eggs_plush_puppies": False,
"first_years_take_and_toss_straw_cup": False,  # need varify
"highland_6539_self_stick_notes": True,
"safety_works_safety_glasses": True,
"genuine_joe_plastic_stir_sticks": True,
"cheezit_big_original": False,
"elmers_washable_no_run_school_glue": False
}
# 
binUseHandVisionList = [9,10,11]

# objectUseHandVisionList = {
# "oreo_mega_stuf": False,
# "crayola_64_ct": False,
# "paper_mate_12_count_mirado_black_warrior": False,
# "mead_index_cards": False,
# "rolodex_jumbo_pencil_cup": False,
# "mark_twain_huckleberry_finn": False,
# "laugh_out_loud_joke_book": False,
# "sharpie_accent_tank_style_highlighters": False,
# "stanley_66_052": False,
# "expo_dry_erase_board_eraser": False,
# "champion_copper_plus_spark_plug": False,
# "feline_greenies_dental_treats": False,
# "kong_air_dog_squeakair_tennis_ball": False,
# "dr_browns_bottle_brush": False,
# "kong_duck_dog_toy": False,
# "kong_sitting_frog_dog_toy": False,
# "munchkin_white_hot_duck_bath_toy": False,
# "mommys_helper_outlet_plugs": False,
# "kyjen_squeakin_eggs_plush_puppies": False,
# "first_years_take_and_toss_straw_cup": False,  # need varify
# "highland_6539_self_stick_notes": False,
# "safety_works_safety_glasses": False,
# "genuine_joe_plastic_stir_sticks": False,
# "cheezit_big_original": False,
# "elmers_washable_no_run_school_glue": False
# }

# binUseHandVisionList = []

def kinect_start():
    kinect_start_=rospy.ServiceProxy('/kinect_start', KinectCommand)
    rospy.sleep(0.1)
    try:
        print '[Percept] kinect_start'
        kinect_start_(1); rospy.sleep(0.1); kinect_start_(2)
    except:
        print '[Percept] kinect_start, Kinect Controller seems not connected, skipping'
    
def kinect_stop():
    kinect_stop_=rospy.ServiceProxy('/kinect_stop', KinectCommand)
    rospy.sleep(0.1)
    try:
        print '[Percept] kinect_stop'
        kinect_stop_(1); rospy.sleep(0.1); kinect_stop_(2)
    except:
        print '[Percept] kinect_stop, Kinect Controller seems not connected, skipping'


def percept(obj_pose = None, binNum = 0, obj_id = None, bin_contents = None, isExecute = True, withPause = True):
    capsen.capsen.init()  
    print '[Percept] percept heuristic'
    
    goToHome.goToHome(robotConfig=None, homePos = [1,0,1.2], isExecute = isExecute, withPause = withPause)
    kinect_obj_pose = None
    
    if useKinect:
        print '[Percept] kinect percept'
        kinect_obj_pose, kinect_score = capsen.capsen.detectOneObject(obj_id, bin_contents, binNum, mode = 0, withScore = True)
        if kinect_obj_pose is None:
            print '[Percept] kinect percept failed.' 
        else:
            kinect_pose_type = find_object_pose_type(obj_id, kinect_obj_pose)
            print '[Percept] kinect percept success.'
        
        if not (objectUseHandVisionList[obj_id] or binNum in binUseHandVisionList or kinect_obj_pose is None):
            return kinect_obj_pose, kinect_pose_type
    
    # turn off kinect
    kinect_stop()
    
    print '[Percept] Perception heuristic wants to do percept_hand.' 
    hand_obj_pose, hand_pose_type, hand_score = percept_hand(obj_pose, binNum, obj_id, bin_contents, isExecute, withPause)
    
    # turn on kinect
    kinect_start()
    
    if hand_obj_pose and kinect_obj_pose: 
        if hand_score >= kinect_score:
            return hand_obj_pose, hand_pose_type
        else:
            return kinect_obj_pose, kinect_pose_type
    elif hand_obj_pose:
        return hand_obj_pose, hand_pose_type
    elif kinect_obj_pose:
        return kinect_obj_pose, kinect_pose_type
    else:
        return None, None

def percept_hand(obj_pose = None, binNum = 0, obj_id = None, bin_contents = None, isExecute = True, withPause = True):
    capsen.capsen.init()
    home_joint_pose = [0, -0.2, 0.2, 0.01, 1, 1.4]
    planner = IKJoint(target_joint_pos=home_joint_pose); 
    #plan = Plan()
    #plan.q_traj = [home_joint_pose]; 
    plan = planner.plan()
    plan.visualize(); 
    plan.setSpeedByName('yolo'); 
    if withPause:
        print '[Percept] Going back to percept_hand home'
        pause()
    plan.execute()
    
    bin_cnstr = get_bin_inner_cnstr()
    
    # 1. init target poses
    target_poses = []
    refind = 7
    for i in range(12):
        target_pose_7_vert = [1.1756, 0, 0.79966] + tfm.quaternion_from_euler(math.pi, 1.4, math.pi).tolist() # for bin 7 vertical
        target_pose_7_hori = [1.1756, 0, 0.79966] + tfm.quaternion_from_euler(2.2, 0, math.pi/2).tolist()
        target_pose_7_invhori = [1.1756, 0, 0.79966] + tfm.quaternion_from_euler(-math.pi/2, 0, -math.pi/2).tolist()
        
        if i < 3:
            target_pose = copy.deepcopy(target_pose_7_invhori)
            if i == 0:
                target_pose[1] -= 0.06
            target_pose[2] -= 0.08  # z
        elif i<6:
            target_pose = copy.deepcopy(target_pose_7_hori)
            target_pose[0] += 0.03  # x
            target_pose[2] -= 0.04  # z
        elif i<9:
            target_pose = copy.deepcopy(target_pose_7_hori)
        else:
            target_pose = copy.deepcopy(target_pose_7_hori)
            target_pose[2] += 0.1  # z

        target_pose[1] += (bin_cnstr[i][0]+bin_cnstr[i][1])/2 - (bin_cnstr[refind][0]+bin_cnstr[refind][1])/2  # y
        target_pose[2] += (bin_cnstr[i][4]+bin_cnstr[i][5])/2 - (bin_cnstr[refind][4]+bin_cnstr[refind][5])/2  # z
        target_poses.append(target_pose)
    
    y_deltas = [0.08, -0.08]
    caption = ['left', 'right']
    max_score = -100
    max_obj_pose = None
    for i in range(len(y_deltas)):
        # 2. plan to target
        tip_hand_transform = xyzrpy_from_xyzquat([-0.067, 0.027, -0.012, -0.007, 0.704, 0.710, -0.002])  # need to be updated
        
        target_pose = copy.deepcopy(target_poses[binNum])
        target_pose[1] += y_deltas[i]
        plan = None
        planner = IK(target_tip_pos = target_pose[0:3], target_tip_ori = target_pose[3:7], 
                     tip_hand_transform=tip_hand_transform, 
                     target_link='link_5', target_joint_bb=[[6, 1.2, 1.2]])
        for j in range(15):
            planner.ori_tol = 0.5*j/10.0
            planner.pos_tol = 0.01*j/10.0
            
            newplan = planner.plan()
            if newplan.success():
                plan = newplan
                print '[Percept] hand planning success at the trial %d' % j
                break
                
        if plan:
            plan.setSpeedByName('fastest')
            plan.visualize()

            if withPause:
                print '[Percept] Going to percept_hand %s ' % caption[i]
                pause()
            plan.execute()
        else:
            print '[Percept] hand planning failed'
            continue
        
        # 3. percept
        rospy.sleep(0.8)  # wait for new pointcloud
        obj_pose, score = capsen.capsen.detectOneObject(obj_id, bin_contents, binNum, mode = 1, withScore = True)
        if score > max_score:
            max_obj_pose = obj_pose
            max_score = score

    # 4. move back to percept home
    planner = IKJoint(target_joint_pos=home_joint_pose)
    plan = planner.plan()
    plan.visualize(); 
    plan.setSpeedByName('yolo'); 
    if withPause:
        print '[Percept] Going back to percept_hand home'
        pause()
    plan.execute()
    
    goToHome.goToHome(robotConfig=None, homePos = [1,0,1.2], isExecute = isExecute, withPause = withPause)
    
    if max_obj_pose is None:
        return None, None, None
    
    pose_type = find_object_pose_type(obj_id, max_obj_pose)
    
    return max_obj_pose, pose_type, max_score

# a helping function for testing without vision
def perceptFromManualFit(obj_id):
    posesrv = rospy.ServiceProxy('/pose_service', GetPose)   # should move service name out
    rospy.sleep(0.5)
    data = posesrv('','')
    
    pose = pose2list(data.pa.object_list[1].pose)
    return (pose, find_object_pose_type(obj_id, pose))
    
if __name__=="__main__":
    rospy.init_node("testingPerceptHand")
    
    # obj_list = ['mommys_helper_outlet_plugs', 
    # 'kong_duck_dog_toy',
    # 'kong_sitting_frog_dog_toy',
    # 'champion_copper_plus_spark_plug',
    # 'mead_index_cards',
    # 'laugh_out_loud_joke_book',
    # 'highland_6539_self_stick_notes',
    # 'elmers_washable_no_run_school_glue',
    # 'stanley_66_052',
    # 'genuine_joe_plastic_stir_sticks',
    # 'safety_works_safety_glasses',
    # 'munchkin_white_hot_duck_bath_toy']
    # 
    # bin_contents_all = [
        # [     "mommys_helper_outlet_plugs", "mark_twain_huckleberry_finn"      ],
        # [        "feline_greenies_dental_treats", "kong_duck_dog_toy"        ],
        # [        "first_years_take_and_toss_straw_cup","kong_sitting_frog_dog_toy"        ],
        # [        "paper_mate_12_count_mirado_black_warrior", "champion_copper_plus_spark_plug"        ],
        # [        "mead_index_cards", "sharpie_accent_tank_style_highlighters"        ],
        # [        "mommys_helper_outlet_plugs", "laugh_out_loud_joke_book"        ],
        # [        "kyjen_squeakin_eggs_plush_puppies", "highland_6539_self_stick_notes"        ],
        # [        "elmers_washable_no_run_school_glue", "champion_copper_plus_spark_plug"        ],
        # [        "crayola_64_ct", "stanley_66_052"        ],
        # [        "genuine_joe_plastic_stir_sticks", "expo_dry_erase_board_eraser"        ],
        # [        "safety_works_safety_glasses"        ],
        # [        "kong_air_dog_squeakair_tennis_ball", "munchkin_white_hot_duck_bath_toy"        ]]
    
    obj_list = ["dr_browns_bottle_brush", "mommys_helper_outlet_plugs", "dr_browns_bottle_brush",
            "first_years_take_and_toss_straw_cup", "mommys_helper_outlet_plugs", "highland_6539_self_stick_notes",
            "mommys_helper_outlet_plugs", "highland_6539_self_stick_notes", "sharpie_accent_tank_style_highlighters",
            "safety_works_safety_glasses", "mead_index_cards", "dr_browns_bottle_brush"]
    
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    for i in range(0,12):
        #pubFrame(br, [0, -0.048, 0, -0.5, 0.5, -0.5, 0.5], 'realsense_depth_optical_frame', 'realsense_link')
        
        print "Trial i=", i
        obj_pose, pose_type = percept(obj_id=obj_list[i], binNum=i, bin_contents = [obj_list[i]], isExecute = True, withPause = False)
        
        
        if obj_pose is None:
            print 'Detection failed'
            continue
        else:
            pubFrame(br, obj_pose, 'obj', 'world')
        
        #pause()
