#!/usr/bin/env python
import rospy
from wsg_50_common.srv import *
from std_srvs.srv import *
import time
import sensor_msgs.msg

ErrorMessage = 'Gripper not connected, skipping gripper command:'

exec_joint_pub = rospy.Publisher('/virtual_joint_states', sensor_msgs.msg.JointState, queue_size=10)
        
        
toRetry = False  # keep retry when motion failed

def move(move_pos=110, move_speed=50):
    #move pos range: 0 - 110 mm
    #move speed range: 5- 450 mm/s
    ack()
    command = 'move'
    srv=rospy.ServiceProxy('/wsg_50_driver/%s' % command, Move)
    #time.sleep(0.05)
    
    while True:
        try:
            error = srv(move_pos, move_speed)
            print '[Gripper] move, return:', error
            break
        except:
            
            print '[Gripper] move,', ErrorMessage, command
            
            # publish to joint state publisher for visualization without real hand
            jnames = ['wsg_50_gripper_base_joint_gripper_left', 'wsg_50_gripper_base_joint_gripper_right']
            
            js = sensor_msgs.msg.JointState()
            js.name  = jnames
            js.position = [-move_pos / 2.0 / 1000.0, move_pos / 2.0 / 1000.0]
            exec_joint_pub.publish(js)
        if not toRetry: break
        time.sleep(0.5)
            

def open(speed=100):
    #we assume that grippers opens fully with speed = 100
    ack()
    move(109, speed)
    # time.sleep(0.1)
    # move(109, speed)
    # time.sleep(0.1)
    # move(109, speed)


def close(speed=100):
    #we assume that grippers closes fully with speed = 100
    ack()
    move(0, speed)
    # time.sleep(0.1)
    # move(0, speed)
    # time.sleep(0.1)
    # move(0, speed)
    
def grasp(move_pos=10, move_speed=80):
    ack()
    command = 'grasp'
    srv=rospy.ServiceProxy('/wsg_50_driver/%s' % command, Move)

    while True:
        try:
            error = srv(move_pos, move_speed)
            print '[Gripper] grasp, return:', error
            break
        except:
            print '[Gripper] grasp,', ErrorMessage, command
        if not toRetry: break
        time.sleep(0.5)

def homing():
    ack()
    command = 'homing'
    srv=rospy.ServiceProxy('/wsg_50_driver/%s' % command, Empty)
    try:
        error = srv()
    except:
        print '[Gripper] homing,', ErrorMessage, command

def ack():
    command = 'ack'
    srv=rospy.ServiceProxy('/wsg_50_driver/%s' % command, Empty)
    try:
        error = srv()
    except:
        print '[Gripper] ack,', ErrorMessage, command
            

def set_force(val = 5):
    command = 'set_force'
    srv=rospy.ServiceProxy('/wsg_50_driver/%s' % command, Conf)
    try:
        error = srv(val)
    except:
        print '[Gripper] set_force,', ErrorMessage, command
            
        

if __name__=='__main__':
    move(50,200)
    
    for i in range(100):
        print 'Trial:', i
        close(50)
        open(50)
        
