#!/usr/bin/env python


import roslib; roslib.load_manifest("interactive_markers")
import rospy
import tf
from manual_fit.srv import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from pr_msgs.msg import *
from tf.broadcaster import TransformBroadcaster
from ik.roshelper import poselist2pose
from ik.roshelper import pose2list
from interactive_markers.interactive_marker_server import *
global brKinect
global brShelf
    
from marker_helper import createMoveControls
from marker_helper import createMeshMarker
from marker_helper import createCubeMarker
from marker_helper import createInteractiveMarker
from ik.roshelper import pubFrame
from ik.helper import xyzrpy_from_xyzquat
from ik.ik import IK
from threading import Thread, Lock

mutex = Lock()
def frameCallback( msg ):
    global br
    global currentTargetPose, currentTargetPoseDirty
    time = rospy.Time.now()
    
    pubFrame(br, pose2list(currentTargetPose), 'target_pose', 'map', npub = 1)
    
    mutex.acquire()
    tmp1 = currentTargetPoseDirty
    tmp2 = currentTargetPose
    currentTargetPoseDirty = False
    mutex.release()
    if tmp1:
        solveIk(pose2list(tmp2))
    

def solveIk(target_pose):
    tip_hand_transform = xyzrpy_from_xyzquat([-0.0053791, 0.0085135, -0.023009, 0.70582, 0.70832, 0.0027417, 0.0095863])
    
    planner = IK(target_tip_pos = target_pose[0:3], target_tip_ori = target_pose[3:7], tip_hand_transform=tip_hand_transform, 
             target_link='link_5', target_joint_bb=[[6, 0, 0]], ori_tol=0.1, pos_tol=0.01, ik_only=True)
    plan = planner.plan()
    
    if plan.success():
        print 'success'
        plan.visualize()
        #plan.execute()  #
    else:
        print 'failed'

def processFeedback(feedback):
    global currentTargetPose, currentTargetPoseDirty
    import rospy
    p = feedback.pose.position
    o = feedback.pose.orientation
    print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)+", "+str(o.x)+", "+str(o.y)+", "+str(o.z)+ ", "+str(o.w)
    
    #if feedback.marker_name == 'target_pose':
    mutex.acquire()
    currentTargetPose = feedback.pose
    currentTargetPoseDirty = True
    mutex.release()
    

if __name__=="__main__":
    rospy.init_node("ik_interactive_server")
    
    global currentTargetPose, currentTargetPoseDirty
    global br
    currentTargetPose = Pose()
    br = TransformBroadcaster()
    currentTargetPoseDirty = False
    
    # create an interactive marker server on the topic namespace ik_interactive
    server = InteractiveMarkerServer("ik_interactive")
    
    # create an interactive marker for TargetPose
    pose = [1.1756, -0.0080496, 0.79966, 0.96262, -0.039399, -0.25934, 0.06743]
    currentTargetPose = poselist2pose( pose )
    int_marker = createInteractiveMarker('target_pose', *pose, frame_id='map')
    # cubemarker = createCubeMarker(offset=tuple(pose[0:3]), rgba=(1,0,1,0.5),
                          # orientation=tuple(pose[3:7]),
                          # scale=(0.01,0.01,0.01))
    #int_marker.controls.append(cubemarker)
    int_marker.controls.extend(createMoveControls(fixed=False))
    server.insert(int_marker, processFeedback)

    rospy.Timer(rospy.Duration(0.1), frameCallback)

    server.applyChanges()
    print 'ready'
    rospy.spin()

