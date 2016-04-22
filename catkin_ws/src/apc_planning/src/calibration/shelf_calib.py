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

def readFromJson(filename):
    with open(filename) as data_file:   
        data = json.load(data_file)
    return data

def main(argv=None):
    rospy.init_node('calib', anonymous=True)
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    
    filename = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/shelf_calib_data/shelf_calib.json'
    robot_touch_floor_pts = readFromJson(filename)
    num_rows = 4
    diff_sum = 0
    print 'floor of the shelves'
    for i in range(num_rows):
        pose = coordinateFrameTransform(robot_touch_floor_pts[i], '/map', '/shelf', listener)
        touch_pt = pose2list(pose.pose)
        (mouthposition, binFloorHeight) = getBinMouthAndFloor(0, i*3)  # a bin at row i
        diff = binFloorHeight - touch_pt[2]  # examine difference in z
        print 'row',i,'(expected - robot_touch)=', diff
        diff_sum += diff
    print 'avg(diff)=', diff_sum/num_rows
    
    filename = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/shelf_calib_data/shelf_calib_left_side.json'
    robot_touch_left_pts = readFromJson(filename)
    num_rows = 4
    diff_sum_left = 0
    print 'left wall of the shelves'
    for i in range(num_rows):
        pose = coordinateFrameTransform(robot_touch_left_pts[i], '/map', '/shelf', listener)
        touch_pt = pose2list(pose.pose)
        (leftWall, rightWall) = getBinSideWalls(i*3)  # a bin at row i
        diff = leftWall - touch_pt[0]  # examine difference in x
        print 'left wall: row',i,'(expected - robot_touch)=', diff
        diff_sum_left += diff
    print 'avg(diff) left wall=', diff_sum_left/num_rows
    
    filename = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/shelf_calib_data/shelf_calib_right_side.json'
    robot_touch_right_pts = readFromJson(filename)
    num_rows = 4
    diff_sum_right = 0
    print 'right wall of the shelves'
    for i in range(num_rows):
        pose = coordinateFrameTransform(robot_touch_right_pts[i], '/map', '/shelf', listener)
        touch_pt = pose2list(pose.pose)
        (leftWall, rightWall) = getBinSideWalls(i*3)  # a bin at row i
        diff = rightWall - touch_pt[0]  # examine difference in x
        print 'right wall: row',i,'(expected - robot_touch)=', diff
        diff_sum_right += diff
    print 'avg(diff) right wall=', diff_sum_right/num_rows
    
    filename = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/shelf_calib_data/shelf_calib_right.json'
    robot_touch_lip_pts = readFromJson(filename)
    num_rows = 4
    diff_sum = 0
    print 'lip to floor of the shelves'
    for i in range(num_rows):
        pose = coordinateFrameTransform(robot_touch_lip_pts[i], '/map', '/shelf', listener)
        touch_pt = pose2list(pose.pose)
        (mouthposition, binFloorHeight) = getBinMouthAndFloor(0, i*3)  # a bin at row i
        diff = binFloorHeight - touch_pt[2]  # examine difference in z
        print 'row',i,'(bin floor - robot_touch)=', diff
        diff_sum += diff
    print 'avg(diff)=', diff_sum/num_rows
    
def getBinSideWalls(FirstBin):
    bin_cnstr = get_bin_cnstr()
    lW = 0
    bin_cnstr_offset =  [[0, -lW, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0],
                         [+lW, 0, 0, 0, 0, 0],
                         [0, -lW, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0],
                         [+lW, 0, 0, 0, 0, 0],
                         [0, -lW, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0],
                         [+lW, 0, 0, 0, 0, 0],
                         [0, -lW, 0, 0, 0, 0],
                         [0, 0, 0, 0, 0, 0],
                         [+lW, 0, 0, 0, 0, 0]]
    new_bin_cnstr = (np.array(bin_cnstr) + np.array(bin_cnstr_offset)).tolist()
    
    leftWall  = new_bin_cnstr[FirstBin+0][1]
    rightWall = new_bin_cnstr[FirstBin+2][0]
    
    return leftWall, rightWall
    

if __name__=='__main__':
    sys.exit(main())
