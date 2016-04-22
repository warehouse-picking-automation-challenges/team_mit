# suction down primitive:

# inputs:
# Pose and position of the object, object ID, shelf pose, position and force
# threshold for grasp.

# fails:
# # The vertical dimension of the object should be smaller than the maximum 
# gap distance between the two fingers.

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
from ik.helper import get_bin_cnstr
from ik.helper import find_object_pose_type
from ik.helper import getBinMouthAndFloor
from ik.helper import getBinMouth
from ik.helper import find_shelf_walls
from ik.helper import pauseFunc
from ik.helper import get_obj_dim
from ik.helper import graspGripper
from ik.helper import setForceGripper
from ik.roshelper import coordinateFrameTransform
from ik.helper import matrix_from_xyzquat
from ik.helper import quat_from_matrix
from ik.helper import rotmatY
from ik.helper import rotmatX
from ik.roshelper import coordinateFrameTransform
from ik.roshelper import pubFrame
from ik.helper import get_bin_inner_cnstr
from ik.helper import openGripper
from ik.helper import closeGripper
from ik.helper import getObjCOM
import goToHome


def suction_side(objPose = [1.95,1.25,1.4,0,0,0,1],
            binNum=4,
            objId = 0,
            bin_contents = None,
            robotConfig = None, 
            shelfPosition = [1.9019,0.00030975,-0.503], 
            forceThreshold = 1, 
            isExecute = True,
            withPause = False):
    ## objPose: world position and orientation frame attached to com of object in quaternion form. XYZ
    ## objId: object identifier
    ## robotConfig: current time robot configuration
    ## shelfPosition: shelf position in world frame
    ## force threshold: amount fo force needed to say we have a grasp
    joint_topic = '/joint_states'  
    ## initialize listener rospy
    listener = tf.TransformListener()
    #rospy.sleep(0.1)
    br = tf.TransformBroadcaster()
    rospy.sleep(0.1)
    
    # shelf variables
    pretensionDelta = 0.00
    lipHeight = 0.035
    
    #tcp_offset_variables
    
    

    
    # plan store
    plans = []
    ## get object position (world frame)
    objPosition = getObjCOM(objPose[0:3], objId)
    obj_pose_tfm_list=matrix_from_xyzquat(objPose[0:3], objPose[3:7])
    obj_pose_tfm=np.array(obj_pose_tfm_list)
    
    obj_pose_orient=obj_pose_tfm[0:3,0:3]
    vertical_index=np.argmax(np.fabs(obj_pose_orient[2,0:3]))
    
    object_dim=get_obj_dim(objId)
    vertical_dim=object_dim[vertical_index]
    
    
    s1=np.fabs(obj_pose_orient[1,0])*object_dim[0]
    s2=np.fabs(obj_pose_orient[1,1])*object_dim[1]
    s3=np.fabs(obj_pose_orient[1,2])*object_dim[2]
    s4=np.fabs(obj_pose_orient[1,vertical_index])*object_dim[vertical_index]
    
    horizontal_dim=s1+s2+s3-s4

    hand_gap=0
    gripper.close()
        
    while True:
        APCrobotjoints = ROS_Wait_For_Msg(joint_topic, sensor_msgs.msg.JointState).getmsg() 
        q0 = APCrobotjoints.position
        if len(q0) < 6:
            continue
        else:
            break
    
        
    ## move gripper to object com outside bin along world y direction and
    ## move the gripper over the lip of the bin    
    # set tcp
    vert_offset=.035
    l2 = 0.43  #change this to edit where you think the cup is
    l2 =.44
    
    tip_hand_transform = [-vert_offset, 0, l2, 0,0,0] # to be updated when we have a hand design finalized
    # broadcast frame attached to tcp
    # for i in range(5):
        # rospy.sleep(0.1)
        # br.sendTransform(tuple(tip_hand_transform[0:3]), tfm.quaternion_from_euler(*tip_hand_transform[3:6]), rospy.Time.now(), 'tip', "link_6")
    # rospy.sleep(0.1)
    
    pubFrame(br, pose=tip_hand_transform, frame_id='target_pose', parent_frame_id='tip', npub=1)
    
    # get position of the tcp in world frame
    pose_world = coordinateFrameTransform(tip_hand_transform[0:3], 'link_6', 'map', listener)
    tcpPos=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    
    
    #this may cause issues!!!
    tcpPosHome = tcpPos
    
    
    # set scoop orientation (rotate wrist)
    
    
    
    
    
    distFromShelf = 0.05
    wristWidth = 0.0725 # this is actually half the wrist width
    (binMouth,bin_height,bin_width) = getBinMouth(distFromShelf, binNum)
    pose_world = coordinateFrameTransform(binMouth[0:3], 'shelf', 'map', listener)
    binMouth=[pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z]
    
    
    
    object_depth=objPosition[0]-binMouth[0]
    
    
    finger_length=.23     
    finger_width=.08
    
    up_offset=.05
    down_offset=.01
    cup_to_spatula=.08
    hand_width=.15
    max_gripper_width=.110
    hand_top_offset=hand_width/2+vert_offset+.015
    hand_bot_offset=hand_width/2-vert_offset+.015
    
    up_down_adjust=.025
    
    side_offset=.03
    
    binFloorHeight=binMouth[2]-bin_height/2
    binCeilHeight=binMouth[2]+bin_height/2
    
    
    min_height=binFloorHeight+lipHeight+finger_width/2+down_offset
    desired_height=objPosition[2]
    max_height=binCeilHeight-lipHeight-finger_width/2-down_offset-up_down_adjust
    
    target_height=desired_height
    if target_height<min_height:
        target_height=min_height
    if target_height>max_height:
        target_height=max_height
    
    bin_sideways=binMouth[1]
    sidepos=objPosition[1]
    
    horz_offset=0.01
    
    #this determines bin sides based on object
    #(binSmall,binLarge)=getSides(objPosition[1],listener)
    (binSmall,binLarge)=getSides(binNum,listener)
    binRight=binSmall
    binLeft=binLarge
    
 
    if sidepos>bin_sideways:
        #this stuff is what happens if the object is to the left
        print '[SucSide] Object is to the left'
        
        #turn the suction cup so that it is sideways left
        scoopOrientation = [.5,-.5,.5,-.5]
        side_waypoint1a=binRight+cup_to_spatula+.01
        side_waypoint1b=sidepos-horizontal_dim/2-2*horz_offset
        if side_waypoint1a>side_waypoint1b:
            side_waypoint1=side_waypoint1a
            print '[SucSide] side waypoint A'
        else:
            side_waypoint1=side_waypoint1b
            print '[SucSide] side waypoint B'
            
        #side_waypoint2=sidepos+horizontal_dim-side_offset
        side_waypoint2=binLeft-horizontal_dim-horz_offset
        
        if side_waypoint2<side_waypoint1:
            if side_waypoint2<side_waypoint1a:
                print '[SucSide] Not enought gap'
                return (False, False)
            else:
                side_waypoint1=side_waypoint2
        

        
        
    else:
        #this stuff is what happens if the object is to the right
        print '[SucSide] Object is to the right'
        
        #turn the suction cup so that it is sideways right
        scoopOrientation = [0.5,.5,.5,.5]
        side_waypoint1a=binLeft-cup_to_spatula-.01
        side_waypoint1b=sidepos+horizontal_dim/2+2*horz_offset
        if side_waypoint1a<side_waypoint1b:
            side_waypoint1=side_waypoint1a
            print '[SucSide] side waypoint A'
        else:
            side_waypoint1=side_waypoint1b
            print '[SucSide] side waypoint B'
        
        #side_waypoint2=sidepos-horizontal_dim+side_offset
        side_waypoint2=binRight+horizontal_dim+horz_offset
        
        if side_waypoint2>side_waypoint1:
            
            if side_waypoint2>side_waypoint1a:
                print '[SucSide] Not enought gap'
                return (False, False)
            else:
                side_waypoint1=side_waypoint2
        


        
    targetPositionList=[    
    [binMouth[0]-.15, side_waypoint1, target_height+up_down_adjust],
    [objPosition[0], side_waypoint1, target_height+up_down_adjust],
    [objPosition[0], side_waypoint2, target_height+up_down_adjust],
    [objPosition[0], side_waypoint2, target_height]]
    
    
    qf=q0
    
    
    for tp_index in range(0, len(targetPositionList)):
    
        targetPosition = targetPositionList[tp_index]
        frontOfObjectPtOutOfBin = targetPosition
        q_initial = qf
        planner = IK(q0 = q_initial, target_tip_pos = targetPosition, target_tip_ori = scoopOrientation, tip_hand_transform=tip_hand_transform, joint_topic=joint_topic)
        plan = planner.plan()
        s = plan.success()
        if s:
            print '[SucSide] move to COM in y successful'
            print '[SucSide] tcp at:', targetPosition
            plan.visualize(hand_param=hand_gap)
            plans.append(plan)
            #if isExecute:
            #    pauseFunc(withPause)
            #    plan.execute()
        else:
            print '[SucSide] move to COM in y fail'
            return (False, False)
            
        #print plan.q_traj
        qf = plan.q_traj[-1]
        print '[SucSide] qf:', qf
    
    #set robot speed
    setSpeedByName(speedName = 'faster')
    for numOfPlan in range(0, len(plans)):
        if numOfPlan >=len(plans)-2:
            setSpeedByName(speedName = 'fast')
        if isExecute:
            plans[numOfPlan].visualize(hand_param=hand_gap)
            pauseFunc(withPause)
            plans[numOfPlan].execute()
            
    
    
    suction.start()
    
    final_hand_gap=110
    
    gripper.set_force(12)
    gripper.grasp(move_pos=final_hand_gap)
    gripper.close()
    
    hand_gap=final_hand_gap
    print '[SucSide] hand_gap:', hand_gap
    
    continue_suction=suction_items(objId)
    
    if continue_suction:
        print '[SucSide] object is of type that suction side will try to remove from bin'
    else:
        print '[SucSide] object is to big to remove from bin'
        
    ## retreat
    for numOfPlan in range(0, len(plans)-1):
        plans[len(plans)-numOfPlan-1].visualizeBackward(hand_param=hand_gap)
        if isExecute:
            pauseFunc(withPause)
            plans[len(plans)-numOfPlan-1].executeBackward()
            if not continue_suction:
                suction.stop()
    
    rospy.sleep(3)
                
    print '[SucSide] Is suction in contact still? Lets see:'
    print '[SucSide] suction.check(): ', suction.check()
    print '[SucSide] suction.check(): ', suction.check()
    
    if suction.check():
        #set robot speed
        print '[SucSide] got item. continuing. suction'
        setSpeedByName(speedName = 'fast')
        return (True, True)
    else:
        print '[SucSide] did not get item. Stopping suction'
        suction.stop()
        return (True, False)
    
def suction_items(objId):
    if objId=='expo_dry_erase_board_eraser':
        return True
    if objId=='stanley_66_052':
        return True
    if objId=='laugh_out_loud_joke_book':
        return True
    if objId=='mark_twain_huckleberry_finn':
        return True
    if objId=='elmers_washable_no_run_school_glue':
        return True
    if objId=='mead_index_cards':
        return True
    if objId=='paper_mate_12_count_mirado_black_warrior':
        return True
    if objId=='sharpie_accent_tank_style_highlighters':
        return True
    
    return False
    
def getCeilHeight(objHeight,listener):
    
    bin_cnstr = get_bin_cnstr()
    
             
    #height_list=[0.800027,1.06673,1.29533,1.52393,1.79063]
    height_list=[bin_cnstr[11][4],bin_cnstr[8][5],bin_cnstr[5][5],bin_cnstr[2][5],bin_cnstr[11][6]]
    
    print '[SucSide] height_list:', height_list
    
    objHeightVec = [0,0,objHeight] # define over the lip distance in shelf frame
    newObjHeightVec = coordinateFrameTransform(objHeightVec, 'map', 'shelf', listener)
    newObjHeight = newObjHeightVec.pose.position.z
    
    print '[SucSide] objHeight:', objHeight
    for i in range(0, len(height_list)-1):
        if (height_list[i]<=newObjHeight) and (newObjHeight<=height_list[i+1]):
            shelf_gap=height_list[i+1]-height_list[i]
            shelfHeightVec=[0,0,height_list[i+1]]
            newShelfHeightVec=coordinateFrameTransform(shelfHeightVec, 'shelf', 'map', listener)
            return (newShelfHeightVec.pose.position.z-shelf_gap,newShelfHeightVec.pose.position.z)
    return '[SucSide] getCeilHeight failed'
    
#def getSides(objX,listener):
             
#    bin_cnstr = get_bin_cnstr()
                 
    #X_list=[-0.42926,-0.1494,0.1554,0.42926]
#    X_list=[bin_cnstr[2][0],bin_cnstr[2][1],bin_cnstr[1][1],bin_cnstr[0][1]]
#    print X_list
    
    #AdjustedX_list=[-0.42926+.045,-0.1494,0.1554,0.42926-.045] #adjust for the lips on far right/left sides
#    AdjustedX_list=[bin_cnstr[2][0]+.045,bin_cnstr[2][1],bin_cnstr[1][1],bin_cnstr[0][1]-.045]
#    print AdjustedX_list
    
#    objXVec = [0,objX,0] 
#    newObjXVec = coordinateFrameTransform(objXVec, 'map', 'shelf', listener)
#    newObjX = newObjXVec.pose.position.x
    

#    for i in range(0, len(X_list)-1):
#        if (X_list[i]<=newObjX) and (newObjX<=X_list[i+1]):
#            shelf_gap=AdjustedX_list[i+1]-AdjustedX_list[i]
#            shelfXVec=[AdjustedX_list[i+1],0,0]
#            
#            newShelfXVec=coordinateFrameTransform(shelfXVec, 'shelf', 'map', listener)
#            
#            return (newShelfXVec.pose.position.y-shelf_gap,newShelfXVec.pose.position.y)
#    return 'failed'

def getSides(binNum,listener):
             
    bin_cnstr = get_bin_cnstr()
                 
    AdjustedX_list=[bin_cnstr[2][0]+.045,bin_cnstr[2][1],bin_cnstr[1][1],bin_cnstr[0][1]-.045]
    print 'AdjustedX_list:', AdjustedX_list
    
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
        suction_side(
        objPose = objPoses[x],
        binNum = binNums[x],
        objId='expo_dry_erase_board_eraser' ,
        robotConfig=None, 
        shelfPosition = [1.9019,0.00030975,-0.503], 
        isExecute = True,
        withPause = False)
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
    # #objPose =[1.55620419979, .281148612499, 1.14214038849,0,0,0,0]
    
    # suction_side(
        # objPose = [pos_x,pos_y,pos_z,orient_x,orient_y,orient_z,orient_w],
        # binNum = 7,
        # objId='expo_dry_erase_board_eraser' ,
        # robotConfig=None, 
        # shelfPosition = [1.9019,0.00030975,-0.503], 
        # isExecute = True,
        # withPause = True)
    # suction.stop()

# objPose =  [1.60593056679, 0.29076179862, 0.863177359104], binNum = 3
# objPose =  [1.55620419979, 0.281148612499, 1.14214038849], binNum = 0, 
# obJPose =  [1.62570548058, 0.289612442255, 0.648919522762], binNum = 6,
