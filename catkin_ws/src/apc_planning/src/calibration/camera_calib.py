#!/usr/bin/env python

import json
import os
from pprint import pprint
from ik.roshelper import coordinateFrameTransform
from ik.helper import getBinMouthAndFloor
from ik.roshelper import pose2list
from ik.helper import get_bin_cnstr
import sys
import rospy
import tf
import numpy as np

from ik.roshelper import pubFrame
from ik.roshelper import poseTransform
from ik.roshelper import ROS_Wait_For_Msg
from apriltags_ros.msg import AprilTagDetectionArray
from ik.roshelper import pose2list
from ik.helper import matrix_from_xyzquat
from ik.helper import pause
import pdb
import tf.transformations as tfm

import math

def readFromJson(filename):
    with open(filename) as data_file:   
        data = json.load(data_file)
    return data

def main(argv=None):
    if argv is None:
        argv = sys.argv
        
    rospy.init_node('calib', anonymous=True)
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(1)
    
    calibtag = 'link6tag1'
    if len(argv) == 2:
        calibtag = argv[1]
    
    # kinect calib joints
    # j1: 9.93
    # j2: 39.46
    # j3: -32.74
    # j4: 69.54
    # j5: -11.26
    # j6: -70.88
    #link6tag1
    #link6tag3
    #link6tag1
    #link6tag3
    # j1: -16.98
    # j2: 65.55
    # j3: -13.48
    # j4: -21.15
    # j5: -51.26
    # j6: 10.63

    #link6tag1
    #link6tag3
    
    #filename = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/camera_extrinsic_calib_data/camera_extrinsic_calib_data.json'
    if calibtag == 'link2tag':
        tag_link_transform = [0, 0.7, -0.2145-0.005, -math.pi, 0, 0]  
        link_frame_id = 'link_2'
    elif calibtag == 'linkbasetag':
        print 'move to q =', (np.array([2.35,-67.73,62.39,-154.9,-30.11,11]) / 180 * math.pi).tolist()
        print 'move to q =', (np.array([2.35,-77.73,72.39,-154.9,-20.11,11]) / 180 * math.pi).tolist()
        tag_link_transform = [0.5291, 0.4428, 0.005, math.pi, 0, 0]  
        link_frame_id = 'base_link'
    elif calibtag == 'link6tag1':
        #print 'move to q =', (np.array([9.93,39.37,-30.63,60.64,-12.12,-61.79]) / 180 * math.pi).tolist()
        tag_link_transform = [-(2.54 + 4 + 1 + 19.23 / 2) / 100.0, 0, 0.01, math.pi, 0, math.pi/2]  
        link_frame_id = 'link_6'
    elif calibtag == 'link6tag2':
        tag_link_transform = [-(2.54 + 4 + 1 + 19.23 / 2) / 100.0, 0, 0.01, math.pi, 0, 0]  
        link_frame_id = 'link_6'
    elif calibtag == 'link6tag3':
        tag_link_transform = [-(2.54 + 4 + 1 + 19.23 / 2) / 100.0, 0, 0.01, math.pi, 0, -math.pi/2]  
        link_frame_id = 'link_6'
    elif calibtag == 'link6tag4':
        tag_link_transform = [-(2.54 + 4 + 1 + 19.23 / 2) / 100.0, 0, 0.01, math.pi, 0, math.pi]  
        link_frame_id = 'link_6'
        
    # visualize it
    pubFrame(br, pose=tag_link_transform, frame_id='tag', parent_frame_id=link_frame_id, npub=10)
    
    
    tag_map_transform = poseTransform(tag_link_transform, link_frame_id, 'map', listener)
    
    apriltag_topic = 'tag_detections' 
    while True:
        tagdetect = ROS_Wait_For_Msg(apriltag_topic, AprilTagDetectionArray).getmsg() 
        if len(tagdetect.detections) == 1:
            tag_kinect_transform = pose2list(tagdetect.detections[0].pose.pose)
            break
    
    tag_kinect_tfm_mat = matrix_from_xyzquat(translate=tag_kinect_transform[0:3], 
                                                     quaternion=tag_kinect_transform[3:7]) 
    tag_map_tfm_mat = matrix_from_xyzquat(translate=tag_map_transform[0:3], 
                                                     quaternion=tag_map_transform[3:7]) 
    
    kinect_map_tfm_mat = np.dot(tag_map_tfm_mat, np.linalg.inv(tag_kinect_tfm_mat))
    
    kinect_map_pose = tfm.translation_from_matrix(kinect_map_tfm_mat).tolist() + tfm.quaternion_from_matrix(kinect_map_tfm_mat).tolist()
    

    link6tag = ['link6tag1', 'link6tag2', 'link6tag3', 'link6tag4']
    if calibtag in link6tag:
        print kinect_map_pose
        pubFrame(br, pose=kinect_map_pose, frame_id='new_kinect_pose', parent_frame_id='map', npub=10)
    else:
        realsense_link5_pose = poseTransform(kinect_map_pose, link_frame_id, 'link_5', listener)
        pubFrame(br, pose=realsense_link5_pose, frame_id='new_realsense_pose', parent_frame_id='link_5', npub=10)
        
        print realsense_link5_pose

    #april_tag_poses_robot = []
    
    #april_tag_pose_camera

if __name__=='__main__':
    sys.exit(main())
    
