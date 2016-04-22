#!/usr/bin/env python

import rospy
import sys
import os
import subprocess
import signal
from apc_perception.srv import *
import traceback

procs = {}

def sigterm_handler(_signo, _stack_frame):
    # Raises SystemExit(0):
    global procs
    for kinect_num, proc in procs.iteritems():
        stop(kinect_num)
    
    sys.exit(0)

def handle_start(req):
    return KinectCommandResponse(start(req.kinect_num))
    
def handle_stop(req):
    return KinectCommandResponse(stop(req.kinect_num))

def stop(kinect_num):
    global procs
    if kinect_num in procs:
        try:
            os.kill(procs[kinect_num].pid, signal.SIGTERM)
            procs.pop(kinect_num, None)
        except:
            print 'stop kinect', kinect_num, 'failed.', traceback.format_exc()
            return 1
    else:
        print 'kinect', kinect_num, 'not started.'
        return 2
    
    return 0
    # delete kinect entry
    
def start(kinect_num):
    global procs
    procs[kinect_num] = subprocess.Popen(["roslaunch", "apc_config", "kinect2_bridge_%d.launch" % kinect_num])

if __name__=='__main__':
    rospy.init_node('kinect_control_server')
    signal.signal(signal.SIGTERM, sigterm_handler)
    start_service = rospy.Service('kinect_start', KinectCommand, handle_start)
    stop_service = rospy.Service('kinect_stop', KinectCommand, handle_stop)
    start(1)
    start(2)
    print "kinect_control_server Ready."
    rospy.spin()


