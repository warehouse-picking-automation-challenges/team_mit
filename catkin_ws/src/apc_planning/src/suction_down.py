#!/usr/bin/env python
# suction down primitive:

# inputs:
# Pose and position of the object, object ID, shelf pose, position and force
# threshold for grasp.

# fails:
# # The vertical dimension of the object should be smaller than the maximum 
# gap distance between the two fingers.

import suction
import geometry_msgs.msg
import std_msgs
import json
import tf
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
from ik.ik import IKJoint
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
from ik.ik import setSpeedByName
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
import goToHome

# put shared function into ik.helper module
from ik.helper import getBinMouth
from ik.roshelper import coordinateFrameTransform
from ik.helper import pauseFunc
from ik.helper import get_obj_dim
from ik.helper import visualizeFunc
from ik.helper import getObjCOM
from ik.helper import openGripper
from ik.helper import closeGripper
from ik.helper import graspGripper
from ik.roshelper import pubFrame
from ik.helper import get_bin_cnstr

def suction_down(objPose = [1.95,1.25,1.4,0,0,0,1],
            binNum=4,
            objId = 0,
            bin_contents = None,
            robotConfig = None, 
            shelfPosition = [1.9019,0.00030975,-0.503], 
            forceThreshold = 1, 
            isExecute = True,
            withPause = True):
    ## objPose: world position and orientation frame attached to com of object in quaternion form. XYZ
    ## objId: object identifier
    ## robotConfig: current time robot configuration
    ## shelfPosition: shelf position in world frame
    ## force threshold: amount fo force needed to say we have a grasp
    joint_topic = '/joint_states'  
    ## initialize listener rospy
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    
    # shelf variables
    pretensionDelta = 0.00
    lipHeight = 0.035
    
    

    

    ## get object position (world frame)
    objPosition = getObjCOM(objPose[0:3], objId)
    obj_pose_tfm_list=matrix_from_xyzquat(objPose[0:3], objPose[3:7])
    obj_pose_tfm=np.array(obj_pose_tfm_list)
    
    obj_pose_orient=obj_pose_tfm[0:3,0:3]
    vertical_index=np.argmax(np.fabs(obj_pose_orient[2,0:3]))
    
    object_dim=get_obj_dim(objId)
    object_dim=adjust_obj_dim(objId,object_dim)
    vertical_dim=object_dim[vertical_index]

    
    
        
    while True:
        APCrobotjoints = ROS_Wait_For_Msg(joint_topic, sensor_msgs.msg.JointState).getmsg() 
        q0 = APCrobotjoints.position
        if len(q0) >= 6:
            q0 = q0[0:6]   # take first 6, because in virtual environmet there will be additional 2 hand joint
            break
    
        
    ## move gripper to object com outside bin along world y direction and
    ## move the gripper over the lip of the bin    
    # set tcp
    vert_offset=.035
    l2 =.44
    
    tip_hand_transform = [-vert_offset, 0, l2, 0,0,0] # to be updated when we have a hand design finalized
    
    # broadcast frame attached to tcp
    pubFrame(br, pose=tip_hand_transform, frame_id='tip', parent_frame_id='link_6', npub=5)
    
    # get position of the tcp in world frame
    pose_world = coordinateFrameTransform(tip_hand_transform[0:3], 'link_6', 'map', listener)
    tcpPos=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    
    
    #this may cause issues!!!
    tcpPosHome = tcpPos
    
    
    # set scoop orientation (rotate wrist)
    scoopOrientation = [0.7071, 0, 0.7071,0]
    
    
    distFromShelf = 0.05
    wristWidth = 0.0725 # this is actually half the wrist width
    (binMouth,bin_height,bin_width) = getBinMouth(distFromShelf, binNum)
    pose_world = coordinateFrameTransform(binMouth[0:3], 'shelf', 'map', listener)
    binMouth=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    
    hand_gap=0
    gripper.close()
    
    finger_length=.23     
    finger_width=.08
    
    up_offset=.05
    down_offset=-.025
    cup_to_spatula=.08
    hand_width=.15
    max_gripper_width=110
    hand_top_offset=hand_width/2+vert_offset+.01
    hand_bot_offset=hand_width/2-vert_offset
    
    
    #this determines bin stuff based on bin input
    binFloorHeight=binMouth[2]-bin_height/2
    binCeilHeight=binMouth[2]+bin_height/2
    
    #this determines bin sides based on object
    #(binSmall,binLarge)=getSides(objPosition[1],listener)
    (binSmall,binLarge)=getSides(binNum,listener)
    
    horz_offset=0    
    small_limit=binSmall+finger_width/2+horz_offset
    large_limit=binLarge-finger_width/2-horz_offset
    
    
    
    
    

    plans = []
    plan = Plan()
    
    if objPosition[1]<0:
        link6_angle=(q0[5]-math.pi)
        possible_start_config=[0.007996209289, -0.6193283503, 0.5283758664, -0.1974229539, 0.09102420595, 3.33888627518-2*math.pi]
    else:
        link6_angle=(q0[5]+math.pi)
        possible_start_config=[0.007996209289, -0.6193283503, 0.5283758664, -0.1974229539, 0.09102420595, 3.33888627518]   
    
    #start_config=possible_start_config
    start_config=[q0[0],q0[1],q0[2],q0[3],q0[4],link6_angle];
    planner = IKJoint(q0 = q0, target_joint_pos=start_config)
    plan = planner.plan()
    #plan.q_traj=[q0,start_config]
    
    plans.append(plan)

    
    qf=plan.q_traj[-1]
    
    
    
    
    
    def get_motion_param(target_x,target_y):
        object_depth=target_x-binMouth[0]
        sidepos=min(max(target_y,small_limit),large_limit)
        
        if object_depth<finger_length:
            print '[SucDown] shallow suction'
            h1=binCeilHeight-lipHeight-cup_to_spatula
            h2=binFloorHeight+vertical_dim-down_offset
        else:
            print '[SucDown] deep suction, quitting'
            h1=binCeilHeight-lipHeight-hand_top_offset
            h2a=binFloorHeight+vertical_dim-down_offset
            h2b=binFloorHeight+hand_bot_offset+lipHeight
            h2=max(h2a,h2b)
            return False, h1,h2,sidepos

        h2=max(h2,binFloorHeight)
                
      
            
        if h2>h1:
            print '[SucDown] cant go in'
            return False, h1,h2,sidepos
        return True,h1,h2,sidepos

    def generate_plan(targetPositionList,plans,qf):
        for tp_index in range(0, len(targetPositionList)):
        
            targetPosition = targetPositionList[tp_index]
            planner = IK(q0 = qf, target_tip_pos = targetPosition, target_tip_ori = scoopOrientation,
            tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
            
            plan = planner.plan()
            s = plan.success()
            print '[SucDown] Plan number:',tp_index+1
            if s:
                print '[SucDown] Plan calculated successfully'
                plans.append(plan)

            else:
                print '[SucDown] Plan calulation failed'
                return (False, plans,qf)
                
            qf = plan.q_traj[-1]
        return (True,plans,qf)
    
    def execute_forward(plans,hand_gap):
        for numOfPlan in range(0, len(plans)):
            if isExecute:
                plans[numOfPlan].visualize(hand_param=hand_gap)
                pauseFunc(withPause)
                plans[numOfPlan].execute()
    
    def execute_backward(plans,hand_gap,plan_offset):
        for numOfPlan in range(0, len(plans)-plan_offset):
            plans[len(plans)-numOfPlan-1].visualizeBackward(hand_param=hand_gap)
            if isExecute:
                pauseFunc(withPause)
                plans[len(plans)-numOfPlan-1].executeBackward()
    
    def try_suction(target_x,target_y,plans,qf,num_iter):

        (continue_val,h1,h2,sidepos)=get_motion_param(target_x,target_y)    
        
        if continue_val==False:
            return False,False
            
            
        final_hand_gap=max_gripper_width
        

        targetPositionList=[
        [target_x, sidepos, h1],
        [target_x, sidepos, h2]]
        
        
        (continue_val,plans,throwaway)=generate_plan(targetPositionList,plans,qf)
        
        if continue_val==False:
            return False,False
            
        #set robot speed
        setSpeedByName(speedName = 'faster')
        
        
        hand_gap=0
        gripper.close()
        
        
        execute_forward(plans,hand_gap)
        
        
        #suction.start()
        
        gripper.set_force(10)
        gripper.grasp(move_pos=max_gripper_width)
        gripper.close()
        hand_gap=max_gripper_width
        
        print '[SucDown] hand_gap:', hand_gap
        
        
        execute_backward(plans,hand_gap,0)
            
                
                
                

        #time.sleep(4)
        my_return=suction.check() or suction_override(objId)
        
        if my_return==True:
            #set robot speed
            setSpeedByName(speedName = 'fast')
            return (True, True)
        else:
            #suction.stop()
            return (True, False)
    
    
    target_offset=get_test_offset(objId)
    target_x_list=[objPosition[0],objPosition[0]-target_offset,objPosition[0],objPosition[0],objPosition[0]+target_offset]
    target_y_list=[objPosition[1],objPosition[1],objPosition[1]+target_offset,objPosition[1]-target_offset,objPosition[1]]
    
    count=0
    
    suction_succeed=False
    
    overall_plan_succeed=False
    
    (continue_val,h1,h2,sidepos)=get_motion_param(target_x=target_x_list[0],target_y=target_y_list[0])    
        
    if continue_val==False:
        return False,False
        
    #targetPositionList=[
    #[binMouth[0]-.15, sidepos, h1],
    #[binMouth[0]-.15, sidepos, h1],
    #[target_x_list[0], sidepos, h1]]
    
    targetPositionListA=[
    [binMouth[0]-.15, sidepos, h1],
    [binMouth[0]-.15, sidepos, h1]]
    
    targetPositionListB=[
    [target_x_list[0], sidepos, h1]]
    
    
    (continue_val,plans1,qf)=generate_plan(targetPositionListA,plans,qf)
    
    if continue_val==False:
        return False,False
        
    (continue_val,plans2,qf)=generate_plan(targetPositionListB,[],qf)
    
    if continue_val==False:
        return False,False
        
    setSpeedByName(speedName = 'yolo')
    execute_forward(plans1,0)
    
    setSpeedByName(speedName = 'faster')
    execute_forward(plans2,0)
    
    suction.start()
    #time.sleep(6)
    
    #plans_store=plans
    plans=[]
                
    for count in range(0,len(target_x_list)):
        
        (plan_succeed,suction_succeed)=try_suction(target_x=target_x_list[count],target_y=target_y_list[count],plans=plans,qf=qf,num_iter=count)
        
        if suction_succeed:
            overall_plan_succeed=True
            break
            #return (plan_succeed,suction_succeed)
            
        if plan_succeed==True:
            overall_plan_succeed=True
        
        while True:
            APCrobotjoints = ROS_Wait_For_Msg(joint_topic, sensor_msgs.msg.JointState).getmsg() 
            qf = APCrobotjoints.position
            if len(qf) < 6:
                continue
            else:
                break
        print '[SucDown] qf:', hand_gap
        
        plans=[]
        count=count+1
    
    execute_backward(plans2,0,0)
    
    if not suction_succeed:
        suction.stop()

    
    return (overall_plan_succeed,suction_succeed)

def suction_override(objId):
    if objId=='mead_index_cards':
        return True
    return False
    
def get_test_offset(objId):
    if objId=='champion_copper_plus_spark_plug':
        return .01
    if objId=='mead_index_cards':
        return .025
    if objId=='kong_air_dog_squeakair_tennis_ball':
        return .053
    if objId=='mommys_helper_outlet_plugs':
        return .05
    return .015
    
def adjust_obj_dim(objId,object_dim):
    if objId=='champion_copper_plus_spark_plug':
        object_dim[0]=object_dim[1]
    if objId=='kong_air_dog_squeakair_tennis_ball':
        object_dim[0]=.03
        object_dim[1]=.03
        object_dim[2]=.03
        
    return object_dim
    


def matrix_from_xyzquat(translate, quaternion):
    return np.dot(tfm.compose_matrix(translate=translate) , 
                   tfm.quaternion_matrix(quaternion)).tolist()

    

    
def getSides(binNum,listener):
             
    bin_cnstr = get_bin_cnstr()
                 
    AdjustedX_list=[bin_cnstr[2][0]+.045,bin_cnstr[2][1],bin_cnstr[1][1],bin_cnstr[0][1]-.045]
    print '[SucDown] AdjustedX_list:', AdjustedX_list
    
    i=2-(binNum%3)
    shelf_gap=AdjustedX_list[i+1]-AdjustedX_list[i]
    shelfXVec=[AdjustedX_list[i+1],0,0]
            
    newShelfXVec=coordinateFrameTransform(shelfXVec, 'shelf', 'map', listener)
            
    return (newShelfXVec.pose.position.y-shelf_gap,newShelfXVec.pose.position.y)
        
    
    
if __name__=='__main__':
    #scoop()
    rospy.init_node('listener', anonymous=True)
    
    suction.stop()
    
    objPoses = []
    objPoses.append([1.599, 0.450, 1.106, -0.0030, 0.6979, -0.7161, 0.0023]) # upper left corner
    objPoses.append([1.607, -0.494, 1.130, -0.0030, 0.6979, -0.7161, 0.0023]) # uppper right corner
    objPoses.append([1.598, 0.467, 0.412, -0.0030, 0.6911, -0.7227, 0.0023]) # lower left corner
    objPoses.append([1.660, -0.500, 0.379, -0.0384, 0.6970, -0.7150, 0.0368]) #lower right corner
    
    binNums = [0,2,9,11]
    
    goToHome.goToHome()
    for x in range(0,4):
        suction_down(
        objPose = objPoses[x],
        binNum = binNums[x],
        objId='expo_dry_erase_board_eraser' ,
        robotConfig=None, 
        shelfPosition = [1.9019,0.00030975,-0.503], 
        isExecute = True,
        withPause = True)
        goToHome.goToHome()
    
    # posesrv = rospy.ServiceProxy('/pose_service', GetPose)   # should move service name out
    # rospy.sleep(0.5)
    # data = posesrv('','')
    # pos_x= data.pa.object_list[0].pose.position.x
    # pos_y= data.pa.object_list[0].pose.position.y
    # pos_z= data.pa.object_list[0].pose.position.z
    # orient_x= data.pa.object_list[0].pose.orientation.x
    # orient_y= data.pa.object_list[0].pose.orientation.y
    # orient_z= data.pa.object_list[0].pose.orientation.z
    # orient_w= data.pa.object_list[0].pose.orientation.w
    
    # suction_down(
        # objPose = [pos_x,pos_y,pos_z,orient_x,orient_y,orient_z,orient_w],
        # binNum = 7,
        # objId='expo_dry_erase_board_eraser' ,
        # robotConfig=None, 
        # shelfPosition = [1.9019,0.00030975,-0.503], 
        # isExecute = True,
        # withPause = True)
    ## suction.stop()

# objPose =  [1.60593056679, 0.29076179862, 0.863177359104], binNum = 3
# objPose =  [1.55620419979, 0.281148612499, 1.14214038849], binNum = 0, 
# obJPose =  [1.62570548058, 0.289612442255, 0.648919522762], binNum = 6,
