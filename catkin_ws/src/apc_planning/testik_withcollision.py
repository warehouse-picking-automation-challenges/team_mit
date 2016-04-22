# a minimal example of using python interface for drake
import geometry_msgs
import std_msgs
import json
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
import tf.transformations as tfm
import rospy
import pdb
import sys
import tf
import math
import numpy as np
import tf.transformations as tfm
target_pose = [0.94, 0, 1.2965,  0, 0.7071, 0, 0.7071]
#target_pose = [0.70, 0-0.2, 1.2965,  0, 0.7071, 0, 0.7071]


l2 = 0.12 + 0.34  # 0.12: gripper base length, 0.34: gripper base to finger tip
tip_hand_transform = [0, 0, l2, 0,0,0] # x,y,z,r,p,y
shelf_world_transform = [1.7831, 0.0036694, -0.50575, 0, 0, 0.70711, 0.70711] # x,y,z,qx,qy,qz,qw
shelf_world_transform = [1.4026, -0.023559, -0.4461, 0, 0, 0.70711, 0.70711] # x,y,z,qx,qy,qz,qw
shelf_world_transform = [1.91, 0, -0.5, 0, 0, 0.70711, 0.70711] # x,y,z,qx,qy,qz,qw

rospy.init_node('listener', anonymous=True)
joint_topic = '/joint_states'
#joint_topic = '/robot1_RosJointState'

# in frame bounding box
inframebb = {}    
def matrix_from_xyzquat(translate, quaternion):
    return np.dot( tfm.compose_matrix(translate=translate) , 
                   tfm.quaternion_matrix(quaternion)).tolist()
inframebb['lb'] = [-100, 0.42, -100] # lowerbound of x,y,z in the frame, -100 is a big number to replace -inf, which is not supported by matlab json decoder
inframebb['ub'] = [100, 100, 100]    # upperbound of x,y,z in the frame
inframebb['frame_mat'] = matrix_from_xyzquat(shelf_world_transform[0:3], shelf_world_transform[3:7])
## 

planner = IK(target_tip_pos = target_pose[0:3], target_tip_ori = target_pose[3:7], 
             joint_topic=joint_topic, tip_hand_transform=tip_hand_transform, straightness = 0.3,
             inframebb = inframebb)
plan = planner.plan()

if plan.success():
    print 'success'
    plan.visualize()
else:
    print 'failed'
    sys.exit()

## visualize the frame
br = tf.TransformBroadcaster()
rospy.sleep(0.5)
br.sendTransform(tuple(tip_hand_transform[0:3]), tfm.quaternion_from_euler(*tip_hand_transform[3:6]), rospy.Time.now(), 'tip', "link_6")
br.sendTransform(tuple(target_pose[0:3]), tuple(target_pose[3:7]), rospy.Time.now(), 'target_pose', "world")

# get user input to visualize again or execute
while True:
    user_reply=raw_input('Execute? [(y)es/(v)isualize/(n)o]: ')
    if user_reply == 'y':
        plan.execute()
        break
    elif user_reply == 'v':
        plan.visualize()
    elif user_reply == 'n':
        break
    
#pdb.set_trace()
#plan.execute()

#raw_input('Pause')

#rostopic pub /ikTrajServer std_msgs/String "'{\"q0\": [-0.006981317007977334, -0.09599310885968834, -0.024434609527920665, 0.015707963267949, 0.905825881785059, -0.01919862177193767], \"target_hand_ori\": [0.7071, 0, 0.7071, 0], \"target_hand_pos\": [0.94, 0, 1.2965]}'"

# {"q0": [-0.006981317007977334, -0.09599310885968834, -0.024434609527920665, 0.015707963267949, 0.905825881785059, -0.01919862177193767], "target_hand_ori": [0.7071, 0, 0.7071, 0], "target_hand_pos": [0.94, 0, 1.2965]}

# data = JSON.parse('{"q0": [-0.006981317007977334, -0.09599310885968834, -0.024434609527920665, 0.015707963267949, 0.905825881785059, -0.01919862177193767], "target_hand_ori": [1, 0, 0, 0], "target_hand_pose": [1, 0, 1]}')

# rosbag play -l $APCDATA_BASE/bagfiles/2015-04-09-18-26-49.bag --clock --rate 0.1 --topics /robot1_RosJointState

# rosbag play -l $APCDATA_BASE/bagfiles/2015-04-09-18-26-49.bag --topics /kinect2/rgb/image

# roslaunch apc_config robot_perception.launch use_realkinect:=false have_robot:=false

# python $APC_BASE/catkin_ws/src/apc_planning/testik.py

# matlabdrake
# matlabapc

# roslaunch launch mcubeAPCSystem.launch


# 1.7831; 0.0036694; -0.50575
# 0; 0; 0.70711; 0.70711
