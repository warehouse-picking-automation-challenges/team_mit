#!/usr/bin/env python
# goToHome primitive:

# description: 
# called when we want to go back home, BE VERY CAREFUL IF CALLED INCORRECTLY
# OR AT A BAD TIME IT MAY HIT THE CAMERAS OR THE SHELF, designed to be used
# from mouth of bins or from the objective bin, not when hand is inside the bin

# the robot moves in essentially a straight line from where it is to the home
# tcp pose (position is taken as argument, pose is assumed to be gripper open
# toward shelf)

# inputs:
# configuration of the robot when called, assumed to be at the mouth bin or
# at objective bin
# location of the home, xyz in world coordinate frame and orientation, this
# is subject to change

import geometry_msgs.msg
import std_msgs
import json
import tf
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
from ik.ik import IKJoint
from ik.ik import Plan
from ik.ik import setSpeedByName
import rospy
import pdb
import numpy as np
import math
import tf.transformations as tfm
import gripper
from ik.helper import getBinMouthAndFloor
from ik.roshelper import coordinateFrameTransform
from ik.helper import pauseFunc
from ik.helper import visualizeFunc
from ik.helper import getObjCOM
from ik.helper import openGripper
from ik.helper import closeGripper

def goToHome(robotConfig = None,
            homePos = [1,0,1.2], 
            isExecute = True,
            withPause = False):
    ## robotConfig: current time robot configuration
    joint_topic = '/joint_states'
    setSpeedByName(speedName = 'yolo')
    
    
    home_joint_pose = [-0.005744439031, -0.6879946105, 0.5861570764, 0.09693312715, 0.1061231983, -0.1045031463]
    planner = IKJoint(target_joint_pos=home_joint_pose)
    plan = planner.plan()
    print ('[goToHome]')
    #print home_joint_pose
    
    plan.visualize()
    if isExecute:
        pauseFunc(withPause)
        plan.execute()
    
    return True
    
    
if __name__=='__main__':
    ## initialize listener rospy
    rospy.init_node('listener', anonymous=True)
    
    goToHome(robotConfig=None,
            homePos = [1,0,1.2], 
            isExecute = True,
            withPause = False)

