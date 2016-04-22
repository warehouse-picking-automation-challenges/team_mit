#!/usr/bin/env python
import geometry_msgs.msg
import std_msgs
import json
import tf
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
import rospy
import pdb
import numpy as np
import math
import tf.transformations as tfm

# define shelf bounds
# the precise bounds gotten from the shelf mesh
# here only the top row is entered
# right \ # left \ # back \  # front  \ # bottom \ # top
# right to left is x axis, bottom to top is z and back to fron is y
bin_cnstr = [[0.1554, 0.42926, 0, 0.42, 1.52393, 1.79063],
             [-0.1494, 0.1494, 0, 0.42, 1.52393, 1.79063],
             [-0.42926, -0.1554, 0, 0.42, 1.52393, 1.79063],
             [0.1554, 0.42926, 0, 0.42, 1.29533, 1.52393],
             [-0.1494, 0.1494, 0, 0.42, 1.29533, 1.52393],
             [-0.42926, -0.1554, 0, 0.42, 1.29533, 1.52393],
             [0.1554, 0.42926, 0, 0.42, 1.06673, 1.29533],
             [-0.1494, 0.1494, 0, 0.42, 1.06673, 1.29533],
             [-0.42926, -0.1554, 0, 0.42, 1.06673, 1.29533],
             [0.1554, 0.42926, 0, 0.42, 0.800027, 1.06673],
             [-0.1494, 0.1494, 0, 0.42, 0.800027, 1.06673],
             [-0.42926, -0.1554, 0, 0.42, 0.800027, 1.06673]]
             
# get points of the mouth of the bins w.r.t. the frame of the shelf
def getBinMouths(bin_cnstr, distFromShelf):
    nrows = len(bin_cnstr)
    ncols = 3
    
    position = [[0 for x in range(ncols)] for x in range(nrows)]
    
    for i in range(nrows):
        x = (bin_cnstr[i][0] + bin_cnstr[i][1])/2
        y = distFromShelf + bin_cnstr[i][3]
        z = (bin_cnstr[i][4] + bin_cnstr[i][5])/2
        position[i][0] = x
        position[i][1] = y
        position[i][2] = z        
        
    return(position)
    
    
if __name__=='__main__':
    rospy.init_node('listener', anonymous=True)
    numBin = 12
    withHand = True
    # call function
    mouthPts = getBinMouths(bin_cnstr, 0.0)

    # apply transform to the points of the mouths of the bin, from shelf
    # frame to robot (world frame)
    listener = tf.TransformListener()
    rospy.sleep(0.1)

    position = []
    
    for i in range(numBin):
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = 'shelf'
        t = listener.getLatestCommonTime("map", "shelf")
        pose.header.stamp = t
        #raw_input('')
        pose.pose.position.x = mouthPts[i][0]
        pose.pose.position.y = mouthPts[i][1]
        pose.pose.position.z = mouthPts[i][2]
        
        #( position, quaternion ) = listener.lookupTransform('map', 'shelf', t)
        #print  position, quaternion 
        
        pose_world = listener.transformPose('map', pose)
        position.append([pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z])

    # plan to each pt (loop of 12, store trajectories in variables) & save
    # plan to a dictionary

    mouthTraj = {}
    rospy.init_node('listener', anonymous=True)

    l1 = 0  
    l2 = 0.12 + 0.34  # +0.05 to grasp between center of fingers
    if withHand:
        tip_hand_transform = [-l1*math.sin(math.pi/6), 0, l1*math.cos(math.pi/6)+l2, 0,0,0] # to be updated when we have a hand design finalized
    else:
        tip_hand_transform = [0,0,0, 0,0,0]
        
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
        
    orient_mat = [[0, 0.7071, 0, 0.7071],[0.5,0.5,0.5,0.5],[0.7071,0,0.7071,0],[-0.5,0.5,-0.5,0.5]]
     
    for binCounter in range(numBin):
        for orient_count in range(4):
            print 'binCounter', binCounter, 'orient_count', orient_count
            for j in range(5):
                rospy.sleep(0.1)
                br.sendTransform(tuple(tip_hand_transform[0:3]), tfm.quaternion_from_euler(*tip_hand_transform[3:6]), rospy.Time.now(), 'tip', "link_6")
            
            # set bin mouth coordinate as target
            # position is transformed of mouthBin variable
            target_xyz = position[binCounter] 
            target_orient = orient_mat[orient_count]
            target_pose = target_xyz + target_orient
            
            for j in range(10):
                rospy.sleep(0.1)
                br.sendTransform(tuple(target_pose[0:3]), tuple(target_pose[3:7]), rospy.Time.now(), 'target_pose', "world")
            
            # call IK planner
            icSearch = 10
            failFlag = False
            for icCounter in range(icSearch):
                print icCounter
                if not(failFlag):
                    qInitialFinalElement = -math.pi+2*math.pi/icSearch*icCounter
                    q_initial = [0,0,0,0,0, qInitialFinalElement]
                    planner = IK(q0 = q_initial, target_tip_pos = target_pose[0:3], target_tip_ori = target_pose[3:7], tip_hand_transform=tip_hand_transform)
                    plan = planner.plan()
                if failFlag:
                    qInitialFinalElement = -math.pi+2*math.pi/icSearch*icCounter
                    #qInitial4thElement = random.uniform(-0.8*math.pi, 0.8*math.pi)
                    qInitial4thElement = 0.1*math.pi
                    q_initial = [0,0,0,qInitial4thElement, 0, qInitialFinalElement]
                    planner = IK(q0 = q_initial, target_tip_pos = target_pose[0:3], target_tip_ori = target_pose[3:7], tip_hand_transform=tip_hand_transform)
                    plan = planner.plan()
                    print 'icCounter reset'
                    
                #pdb.set_trace()
                #~ planner = IK(target_tip_pos = target_pose[0:3], target_tip_ori = target_pose[3:7], tip_hand_transform=tip_hand_transform)
                #~ plan = planner.plan()
                #
                s = plan.success()
                if s:
                    print 'success 1'
                    plan.visualize()
                    break
                else:
                    if icCounter == icSearch-1 and ~failFlag:
                        failFlag = True
                        icCounter = 0                            
                    if icCounter == icSearch-1 and failFlag:
                        print 'failed 1'
                    if icCounter < icSearch-1:
                        continue
            
            qf = plan.q_traj[-1] 
            
            
            target_pose = (np.array(target_pose) + np.array([0.42, 0, 0, 0, 0, 0, 0])).tolist()
            
            for j in range(10):
                rospy.sleep(0.1)
                br.sendTransform(tuple(target_pose[0:3]), tuple(target_pose[3:7]), rospy.Time.now(), 'target_pose2', "world")
            
            planStroke = IK(q0 = qf, target_tip_pos = target_pose[0:3], target_tip_ori = target_pose[3:7], tip_hand_transform=tip_hand_transform, straightness = 0.99)
            plan_stroke = planStroke.plan()
            
            #pdb.set_trace()
            s = plan_stroke.success()
            if s:
                print 'success 2'
                plan_stroke.visualize()
            else:
                print 'failed 2'
            
            # add to dictionary
            mouthTraj[binCounter] = plan
            
