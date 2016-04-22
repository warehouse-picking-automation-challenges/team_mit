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
    
    leftWall  = new_bin_cnstr[FirstBin][1]
    rightWall = new_bin_cnstr[FirstBin][0]
    
    return leftWall, rightWall

def main(argv=None):
    rospy.init_node('calib', anonymous=True)
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    
    filename = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/shelf_calib_data/shelf_calib_all_Final_apc.json'
    robot_touch_pts = readFromJson(filename)
    
    binNums = [0,2,3,5,6,8,9,11]
    nbin = len(binNums)
    diff_sum = 0
    # shelf_pose/x: 1.92421
    # shelf_pose/y: -0.0124050312627
    # shelf_pose/z: -0.513553368384
    ###########################################
    print 'floor of the shelves'
    for i in binNums:
        pose = coordinateFrameTransform(robot_touch_pts[i][4], '/map', '/shelf', listener)
        touch_pt = pose2list(pose.pose)
        (mouthposition, binFloorHeight) = getBinMouthAndFloor(0, i)  # a bin at row i
        diff = binFloorHeight - touch_pt[2]  # examine difference in z
        print 'bin',i,'(expected - robot_touch)=', diff
        diff_sum += diff
    print 'avg(diff)=', diff_sum/nbin
    offset_x = diff_sum/nbin
    print 'Please move the shelf in z by', offset_x
    print '\twhich is', -0.513553368384 - offset_x
    
    ###########################################
    diff_sum_left = 0
    print 'left wall of the shelves'
    for i in binNums:
        pose = coordinateFrameTransform(robot_touch_pts[i][1], '/map', '/shelf', listener)
        touch_pt = pose2list(pose.pose)
        (leftWall, rightWall) = getBinSideWalls(i)  # a bin at row i
        diff = leftWall - touch_pt[0]  # examine difference in x
        print 'left wall: bin',i,'(expected - robot_touch)=', diff
        diff_sum_left += diff
    print 'avg(diff) left wall=', diff_sum_left/nbin
    
    
    ###########################################
    diff_sum_right = 0
    print 'right wall of the shelves'
    for i in binNums:
        pose = coordinateFrameTransform(robot_touch_pts[i][2], '/map', '/shelf', listener)
        touch_pt = pose2list(pose.pose)
        (leftWall, rightWall) = getBinSideWalls(i)  # a bin at row i
        diff = rightWall - touch_pt[0]  # examine difference in x
        print 'right wall: bin',i,'(expected - robot_touch)=', diff
        diff_sum_right += diff
    print 'avg(diff) right wall=', diff_sum_right/nbin
    
    offset_y = (diff_sum_left/nbin+diff_sum_right/nbin) / 2
    print 'Please move the shelf in y by', offset_y
    print '\twhich is', 0.0124050312627 - offset_y
    
    ###########################################
    diff_sum = 0
    print 'lip to floor of the shelves'
    for i in binNums:
        pose = coordinateFrameTransform(robot_touch_pts[i][0], '/map', '/shelf', listener)
        touch_pt = pose2list(pose.pose)
        (mouthposition, binFloorHeight) = getBinMouthAndFloor(0, i)  # a bin at row i
        diff = binFloorHeight - touch_pt[2]  # examine difference in z
        print 'row',i,'(bin floor - robot_touch)=', diff
        diff_sum += diff
    print 'avg(diff)=', diff_sum/nbin

    offset_y = (diff_sum_left/nbin+diff_sum_right/nbin) / 2
    print 'Please move the shelf in y by', offset_y
    print '\twhich is', 0.0124050312627 - offset_y



    ##########################################  front
    diff_sum_front = 0
    print 'front of the shelves'
    for i in binNums:
        pose = coordinateFrameTransform(robot_touch_pts[i][2], '/map', '/shelf', listener)
        touch_pt = pose2list(pose.pose)
        (leftWall, rightWall) = getBinSideWalls(i)  # a bin at row i
        diff = rightWall - touch_pt[1]  # examine difference in y
        print 'right front wall: bin',i,'(expected - robot_touch)=', diff
        diff_sum_front += diff
    print 'avg(diff) right wall=', diff_sum_front/nbin
    offset_x = diff_sum_front/nbin
    print 'Please move the shelf in x by', offset_x
    print '\twhich is', 1.92421 - offset_x  #- 0.87/2 

    

if __name__=='__main__':
    sys.exit(main())
