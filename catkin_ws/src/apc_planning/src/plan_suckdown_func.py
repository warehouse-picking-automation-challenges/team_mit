import rospy
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import json
import visualization_msgs.msg
import trajectory_msgs.msg
import moveit_msgs.msg
import telnetlib
import tf.transformations as tfm
import numpy as np
import roslib; roslib.load_manifest("robot_comm")
from robot_comm.srv import *
import time
import pdb
import socket
import math
from visualization_msgs.msg import *
import os
import sys
import tf
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
from ik.helper import find_object_pose_type
import rospy
import pdb
import math
import gripper

def plan_suckdown_func(plan = None): 
    joint_topic = '/joint_states',
    while True:
        APCrobotjoints = ROS_Wait_For_Msg(joint_topic, sensor_msgs.msg.JointState).getmsg() 
        q0 = APCrobotjoints.position
        if len(q0) < 6:
            continue
        else:
            argin['q0'] = q0
            break
        plan_suckdown = plan()
        for n in range(len(plan.q_traj)):
            plan_suckdown.q_traj.append(plan.q_traj([n][0:4]).extend(q0[5]))
        
    return plan_suckdown
        
