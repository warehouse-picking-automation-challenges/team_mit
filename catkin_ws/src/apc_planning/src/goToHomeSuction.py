#!/usr/bin/env python
import suction
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

def goToHomeSuction(plan = None):
    
    joint_topic = '/joint_states'  
    ## initialize listener rospy
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    
    while True:
        APCrobotjoints = ROS_Wait_For_Msg(joint_topic, sensor_msgs.msg.JointState).getmsg() 
        q0 = APCrobotjoints.position
        if len(q0) < 6:
            continue
        else:
            break
            
    plan_n = Plan()
    
    if q0[5]<0:
        possible_start_config=[0.007996209289, -0.6193283503, 0.5283758664, -0.1974229539, 0.09102420595, 3.33888627518-2*math.pi]
    else:
        possible_start_config=[0.007996209289, -0.6193283503, 0.5283758664, -0.1974229539, 0.09102420595, 3.33888627518]   
    
    plan_n.q_traj=[possible_start_config]
    
    return plan_n
    
if __name__=='__main__':
    rospy.init_node('listener', anonymous=True)
    
    goToHomeSuction(plan = None)
