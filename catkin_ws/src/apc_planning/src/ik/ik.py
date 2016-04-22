#!/usr/bin/env python

import rospy
import geometry_msgs.msg
import std_msgs.msg
import sensor_msgs.msg
import json
from roshelper import ROS_Wait_For_Msg
import roshelper
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

from helper import graspGripper
from helper import moveGripper
from helper import setForceGripper
#~ import gripper
#~ import suction

toviz = False
haverobot = True

class IK:
    # target_hand_pos = [x, y, z] (required)
    # target_hand_ori = [qx, qy, qz, qw] (required)
    # tip_hand_transform = [x,y,z, r,p,y]
    # target_joint_bb = [[joint_ind, lb, ub], [joint_ind, lb, ub], ...]  joint_ind: 1-6, don't use inf, use a big number instead.
    # from what time should it follow the orientation constraint of target
    
    # q0 = 6 joint angles in rad
    def __init__(self, joint_topic = '/joint_states', target_tip_pos = None, target_tip_ori = None, q0 = None, 
                 straightness = 0, pos_tol = 0.001, ori_tol = 0.01,
                 tip_hand_transform = [0,0,0, 0,0,0], inframebb = None,
                 target_link = 'link_6', target_joint_bb = None,
                 N = 10, ik_only = False):
        self.q0 = q0
        self.target_tip_pos = target_tip_pos
        self.target_tip_ori = target_tip_ori
        self.pos_tol = pos_tol
        self.ori_tol = ori_tol
        self.tip_hand_transform = tip_hand_transform
        self.straightness = straightness  
        self.inframebb = inframebb
        self.joint_topic = joint_topic
        self.target_link = target_link
        self.target_joint_bb = target_joint_bb
        
        self.N = N
        self.ik_only = ik_only
        self.ikServerAddress = ('localhost', 30000)
        
    def plan(self):
        # plan() return an object includes attributes
        #   q_traj,
        #   snopt_info_iktraj, 
        #   infeasible_constraint_iktraj 
        #   snopt_info_ik
        
        # 1. Get a handle for the service
        #    todo: makes warning if ikTrajServer does not exist
        ikTrajServer_pub = rospy.Publisher('/ikTrajServer', std_msgs.msg.String, queue_size=100)
        
        argin = {}
        # 2. prepare input arguments        
        if self.q0 is not None: 
            argin['q0'] = self.q0
        else:
            # get robot joints from published topic  # sensor_msgs/JointState
            while True:
                APCrobotjoints = ROS_Wait_For_Msg(self.joint_topic, sensor_msgs.msg.JointState).getmsg() 
                q0 = APCrobotjoints.position
                if len(q0) < 6:
                    continue
                else:
                    argin['q0'] = q0
                    break
            
        # 2.1 transform tip pose to hand pose    
        if self.target_tip_pos is not None: 
            tip_hand_tfm_mat = np.dot( tfm.compose_matrix(translate=self.tip_hand_transform[0:3]),
                                       tfm.compose_matrix(angles=self.tip_hand_transform[3:6]) )
            tip_world_rot_mat = tfm.quaternion_matrix(self.target_tip_ori)
            tip_world_tfm_mat = np.dot(tfm.compose_matrix(translate=self.target_tip_pos) , tip_world_rot_mat)
            hand_tip_tfm_mat = np.linalg.inv(tip_hand_tfm_mat)
            hand_world_tfm_mat = np.dot(tip_world_tfm_mat, hand_tip_tfm_mat)
            target_hand_pos = tfm.translation_from_matrix(hand_world_tfm_mat)
            
            argin['target_hand_pos'] =  target_hand_pos.tolist()
            
        if self.target_tip_ori is not None: 
            tip_hand_tfm_mat = tfm.compose_matrix(angles=self.tip_hand_transform[3:6])
            tip_world_rot_mat = tfm.quaternion_matrix(self.target_tip_ori)
            hand_tip_tfm_mat = np.linalg.inv(tip_hand_tfm_mat)
            hand_world_tfm_mat = np.dot(tip_world_tfm_mat, hand_tip_tfm_mat)
            target_hand_ori = tfm.quaternion_from_matrix(hand_world_tfm_mat)
            
            argin['target_hand_ori'] = roshelper.ros2matlabQuat(target_hand_ori.tolist())  # tolist: so that its serializable
        
        # 2.2 prepare other options
        argin['straightness'] = self.straightness
        argin['target_link'] = self.target_link
        argin['pos_tol'] = self.pos_tol
        argin['ori_tol'] = self.ori_tol
        
        if self.inframebb is not None:
            argin['inframebb'] = self.inframebb
        
        if self.target_joint_bb is not None:
            argin['target_joint_bb'] = self.target_joint_bb
        
        argin['N'] = self.N
        
        argin['tip_hand_transform'] = self.tip_hand_transform
        
        if self.ik_only:
            argin['ik_only'] = 1
        else:
            argin['ik_only'] = 0
        
        argin_json = json.dumps(argin)                    # convert to json format and 
        
        
        for i in range(10):
            # 3. call it
            #print 'calling matlab drake...'
            argin_json_ = argin_json + '\n'
            
            try:
                tn = telnetlib.Telnet(self.ikServerAddress[0], self.ikServerAddress[1])
                
                time.sleep(0.05)
                tn.write(argin_json_)
            except socket.error as inst:
                print '[Drake] Matlab server not up now.'
                if i==9:
                    return None
                continue
        
            time.sleep(0.05)
            
            # 4. parse output arguments
            try:
                ret_json = tn.read_all()
                ret = json.loads(ret_json)  # convert from json to struct
                break
            except:
                print '[Drake] read_all() failed, try again.'
                if i==9:
                    return None
                continue
        
        return Plan(ret)
        


class EvalPlan:
    def __init__(self, script = None):
        self.script = script
        
    def execute(self):
        eval(self.script)
        
    def visualize(self,hand_param=.055):
        pass
        
    def visualizeBackward(self,hand_param=.055):
        pass
        
    def executeBackward(self):
        pass

class IKJoint:
    def __init__(self, target_joint_pos, joint_topic = '/joint_states', q0 = None):
        self.target_joint_pos = target_joint_pos
        self.q0 = q0
        if self.q0 is not None: 
            self.q0 = q0[0:6]
        self.joint_topic = joint_topic
        
    def plan(self):
        N = 100
        p = Plan()
             
        if self.q0 is None: 
            # get robot joints from published topic  # sensor_msgs/JointState
            while True:
                APCrobotjoints = ROS_Wait_For_Msg(self.joint_topic, sensor_msgs.msg.JointState).getmsg() 
                q0 = APCrobotjoints.position
                if len(q0) < 6:
                    continue
                else:
                    self.q0 = q0[0:6]
                    break
        
        for i in range(N):
            t = (i*1.0)/(N-1)
            q_t = (np.array(self.target_joint_pos)*t + np.array(self.q0)*(1-t)).tolist()
            p.q_traj.append(q_t)
        
        return p
        


class Plan:
    def __init__(self, data = None):
        self.q_traj = []
        self.data = None
        if data is not None:
            self.data = data
            self.q_traj = np.array(data['q_traj']).transpose().tolist()
        self.ikTrajServer_pub = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        self.exec_joint_pub = rospy.Publisher('/virtual_joint_states', sensor_msgs.msg.JointState, queue_size=10)
        self.speedName = None
        self.speed = None  # should be a tuple: (tcpSpeed, oriSpeed); None means don't change speed
    
    def setSpeedByName(self, speedName = 'fast'):
        # superSaiyan, yolo, fastest, faster, fast, slow
        name2vel = {'superSaiyan': (1200,120),'yolo': (800,90),'fastest': (400,70),'faster': (200,60),
                    'fast': (100,30), 'slow': (50,15)}
        self.speedName = speedName
        self.speed = name2vel[speedName]
    
    def success(self):
        if self.data is not None:
            return (self.data['snopt_info_iktraj'] < 10 and self.data['snopt_info_ik'] < 10)
        return True
    
    def _visualize(self, backward=False, hand_param=None):
        if not toviz:
            return
        if hand_param is None:
            joint_topic = '/joint_states'
            hand_param = 0.11
            for i in range(100):
                APCrobotjoints = ROS_Wait_For_Msg(joint_topic, sensor_msgs.msg.JointState).getmsg() 
                q0 = APCrobotjoints.position
                if len(q0) == 8 or len(q0) == 2:
                    hand_q = q0[-2:]
                    hand_param = math.fabs(hand_q[0])
                    break
        
        jointTrajMsg = trajectory_msgs.msg.JointTrajectory()
        q_traj = self.q_traj  # a N-by-6 matrix
        
        jointTrajMsg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 
        'joint5', 'joint6', 'wsg_50_gripper_base_joint_gripper_left', 'wsg_50_gripper_base_joint_gripper_right']
        
        speedup = 3
        if backward:
            rng = range(len(q_traj)-1, -1, -speedup) + [0]
        else:
            rng = range(0, len(q_traj), speedup) + [len(q_traj)-1]
        
        for i in rng:
            pt = trajectory_msgs.msg.JointTrajectoryPoint()
            for j in range(6):
                pt.positions.append(q_traj[i][j])
            pt.positions.append(-hand_param)  # open gripper, these two numbers should be planned somewhere
            pt.positions.append(hand_param)
            jointTrajMsg.points.append(pt)

        robotTrajMsg = moveit_msgs.msg.RobotTrajectory()
        robotTrajMsg.joint_trajectory = jointTrajMsg

        dispTrajMsg = moveit_msgs.msg.DisplayTrajectory()
        dispTrajMsg.model_id = 'irb_1600id'
        dispTrajMsg.trajectory.append(robotTrajMsg)
        
        #rospy.sleep(0.1)
        self.ikTrajServer_pub.publish(dispTrajMsg)
        #rospy.sleep(0.1)

    def visualize(self,hand_param=None):
        self._visualize(backward=False,hand_param=hand_param)
        
    def visualizeForward(self,hand_param=None):
        self._visualize(backward=False,hand_param=hand_param)
        
    def visualizeBackward(self,hand_param=None):
        self._visualize(backward=True,hand_param=hand_param)
        
    def _execute(self, backward=False):
        if self.speed is not None:
            setSpeed(self.speed[0], self.speed[1])
        
        clearBuffer = rospy.ServiceProxy('/robot1_ClearJointPosBuffer', robot_ClearJointPosBuffer)   # should move service name out
        addBuffer = rospy.ServiceProxy('/robot1_AddJointPosBuffer', robot_AddJointPosBuffer)
        executeBuffer = rospy.ServiceProxy('/robot1_ExecuteJointPosBuffer', robot_ExecuteJointPosBuffer)
        
        q_traj = self.q_traj  # a N-by-6 matrix
        if backward:
            rng = range(len(q_traj)-1, -1, -10) + [0]
        else:
            rng = range(0, len(q_traj), 10) + [len(q_traj)-1]
        R2D = 180.0 / math.pi
        
        try:
            if not haverobot:
                raise Exception('No robot')
            rospy.wait_for_service('/robot1_ClearJointPosBuffer', timeout = 0.5)
            clearBuffer()
            
            for j in rng:
                addBuffer(q_traj[j][0]*R2D, q_traj[j][1]*R2D, q_traj[j][2]*R2D,
                 q_traj[j][3]*R2D, q_traj[j][4]*R2D, q_traj[j][5]*R2D)
            ret = executeBuffer()
            
            if ret.ret == 1:
                return True
            else:             
                print '[Robot] executeBuffer() error:', ret.ret, ret.msg
                return False

        except:
            print '[Robot] Robot seeems not connected, skipping execute()'
            
            jnames = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
            js = sensor_msgs.msg.JointState()
            for j in rng:
                js.name  = jnames
                js.position = q_traj[j]
                self.exec_joint_pub.publish(js)
                rospy.sleep(0.05)
            
            return True  # should be something else, set True so that primitive can run without robot

    def execute(self):
        return self._execute(backward=False)
        
    def executeForward(self):
        return self._execute(backward=False)
        
    def executeBackward(self):
        return self._execute(backward=True)


def setSpeedByName(speedName = 'fast'):
    # superSaiyan, yolo, fastest, faster, fast, slow
    name2vel = {'superSaiyan': (1200,120),'yolo': (800,90),'fastest': (400,70),'faster': (200,60),
                'fast': (100,30), 'slow': (50,15)}
    speed = name2vel[speedName]
    setSpeed(speed[0], speed[1])

def setSpeed(tcp=100, ori=30):
    if not haverobot:
        return
        
    setSpeed_ = rospy.ServiceProxy('/robot1_SetSpeed', robot_SetSpeed)
    
    print '[Robot] setSpeed(%.1f, %.1f)' % (tcp, ori)
    try:
        rospy.wait_for_service('/robot1_SetSpeed', timeout = 0.5)
        setSpeed_(tcp, ori)
        return True
    except:
        print '[Robot] Robot seeems not connected, skipping setSpeed()'
        return False
