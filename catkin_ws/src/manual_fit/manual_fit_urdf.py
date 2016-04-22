#!/usr/bin/env python

import roslib; roslib.load_manifest("interactive_markers")
import rospy
import tf
from manual_fit.srv import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from pr_msgs.msg import *
from tf.broadcaster import TransformBroadcaster
import math

from interactive_markers.interactive_marker_server import *

from marker_helper import createMeshMarker
from marker_helper import createCubeMarker
from marker_helper import createMoveControls
from marker_helper import createInteractiveMarker

    
def frameCallbackLinks( msg ):
    global brLink4, brLink5, brLink6, brLink_tag
    global br
    global currentJoint6, currentJoint5, currentJoint4, currentJoint_tag
    time = rospy.Time.now()
    
    p = currentJoint_tag.position
    o = currentJoint_tag.orientation
    brLink_tag.sendTransform( (p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
    time, "tag", "link_2" )
    
    p = currentJoint6.position
    o = currentJoint6.orientation
    brLink6.sendTransform( (p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
    time, "link_6x", "link_5x" )
    
    p = currentJoint5.position
    o = currentJoint5.orientation
    brLink5.sendTransform( (p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
    time, "link_5x", "link_4x" )
    
    p = currentJoint4.position
    o = currentJoint4.orientation
    brLink4.sendTransform( (p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
    time, "link_4x", "link_3" )


def processFeedback(feedback):
    global currentJoint4, currentJoint5, currentJoint6, currentJoint_tag
    import rospy
    p = feedback.pose.position
    o = feedback.pose.orientation
    #print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)+", "+str(o.x)+", "+str(o.y)+", "+str(o.z)+ ", "+str(o.w)
    rpy = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
    print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)+", "+str(rpy[0])+", "+str(rpy[1])+", "+str(rpy[2])
    
    if feedback.marker_name == 'joint4':
        currentJoint4 = feedback.pose
    if feedback.marker_name == 'joint5':
        currentJoint5 = feedback.pose
    if feedback.marker_name == 'joint6':
        currentJoint6 = feedback.pose
    if feedback.marker_name == 'joint_tag':
        currentJoint_tag = feedback.pose

# usage: 1. put in the parameters in the current urdf
#        2. active the interactive marker associated to the link/joint that you're tuning (uncomment the server.insert)
#        3. roslaunch apc_config display_urdf.launch model:=irb_1600id.urdf gui:=True
#        4. run this script
#        5. adjust the link mesh / joint in rviz
#        6. put in the correct tuned parameters in this script
#        7. adjust iteratively
#        8. put the correct parameters to the urdf file
#        9. to update urdf in rviz: rosparam set /robot_description --textfile irb_1600id.urdf
#        10. uncheck and check robot_model in rviz to refresh

if __name__=="__main__":
    rospy.init_node("manual_fit_urdf_server")
    
    global link4, link5, link6
    global brLink4, brLink5, brLink6, brLink_tag
    global currentJoint4, currentJoint5, currentJoint6
    global currentJoint_tag
    currentJoint4 = Pose()
    currentJoint5 = Pose()
    currentJoint6 = Pose()
    currentJoint_tag = Pose()
    brLink4 = TransformBroadcaster()
    brLink5 = TransformBroadcaster()
    brLink6 = TransformBroadcaster()
    brLink_tag = TransformBroadcaster()
    
    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("urdf_marker")
    
    # create an interactive marker for finger_left    
    trans = (0.0658272579312, -0.0897141247988, 0.256312400103) 
    qlink = tf.transformations.quaternion_from_euler(-math.pi/2, math.pi/2, 0)  # put in mesh vs joint
    int_marker = createInteractiveMarker('finger_left', 
    trans[0], trans[1], trans[2], qlink[0], qlink[1], qlink[2], qlink[3], frame_id='finger_left')
    meshmarker = createMeshMarker('package://apc_config/models/wsg_50/meshes/SpatulaFingersTop.stl', 
                              offset=trans, rgba=(0,1,0,1),
                              orientation=(qlink[0], qlink[1], qlink[2], qlink[3]), frame_id='/finger_left', scale=0.001) 
    int_marker.controls.append(meshmarker)
    int_marker.controls.extend(createMoveControls())
    #server.insert(int_marker, processFeedback)
    
    # create an interactive marker for finger_right  
    trans = (-0.0974799394608, -0.131322726607, 0.111145801842) 
    qlink = tf.transformations.quaternion_from_euler(-math.pi/2, -math.pi/2, 0)
    int_marker = createInteractiveMarker('finger_right', 
    trans[0], trans[1], trans[2], qlink[0], qlink[1], qlink[2], qlink[3], frame_id='finger_right')
    meshmarker = createMeshMarker('package://apc_config/models/wsg_50/meshes/SpatulaFingersBottom.stl', 
                              offset=trans, rgba=(0,1,0,1),
                              orientation=(qlink[0], qlink[1], qlink[2], qlink[3]), frame_id='/finger_right', scale=0.001) 
    int_marker.controls.append(meshmarker)
    int_marker.controls.extend(createMoveControls())
    server.insert(int_marker, processFeedback)

    # create an interactive marker for wrist  
    trans = (-0.0726601332426, 0.0423464477062, 0.119952730834)
    qlink = tf.transformations.quaternion_from_euler(math.pi, 0, 0)
    frame_id = 'link_wrist'
    int_marker = createInteractiveMarker(frame_id, 
    trans[0], trans[1], trans[2], qlink[0], qlink[1], qlink[2], qlink[3], frame_id=frame_id)
    meshmarker = createMeshMarker('package://apc_config/models/IRB1600ID/meshes/visual/link_wrist.stl', 
                              offset=trans, rgba=(0,1,0,1),
                              orientation=(qlink[0], qlink[1], qlink[2], qlink[3]), frame_id=frame_id, scale=0.001) 
    int_marker.controls.append(meshmarker)
    int_marker.controls.extend(createMoveControls())
    #server.insert(int_marker, processFeedback)

    # create an interactive marker for all 
    qlink = tf.transformations.quaternion_from_euler(math.pi/2, math.pi/2, math.pi)
    trans = (0.073, -0.025, 0) 
    int_marker = createInteractiveMarker('hand_all', 
    trans[0], trans[1], trans[2], qlink[0], qlink[1], qlink[2], qlink[3], frame_id='hand_all')
    meshmarker = createMeshMarker('package://apc_config/models/wsg_50/meshes/all_hand.stl', 
                              offset=trans, rgba=(1,0,0,1),
                              orientation=(qlink[0], qlink[1], qlink[2], qlink[3]), frame_id='/hand_all', scale=0.001) 
    int_marker.controls.append(meshmarker)
    int_marker.controls.extend(createMoveControls())
    #server.insert(int_marker, processFeedback)

    # create an interactive marker for joint4 
    currentJoint4.position.x = 0      # put in joint vs joint
    currentJoint4.position.y = 0.11
    currentJoint4.position.z = 0
    qjoint = tf.transformations.quaternion_from_euler(0, -math.pi/2, 0)
    currentJoint4.orientation.x = qjoint[0]
    currentJoint4.orientation.y = qjoint[1]
    currentJoint4.orientation.z = qjoint[2]
    currentJoint4.orientation.w = qjoint[3]
    p = currentJoint4.position
    o = currentJoint4.orientation
    int_marker = createInteractiveMarker('joint4',p.x, p.y, p.z, o.x, o.y, o.z, o.w, frame_id='link_3')
    int_marker.controls.extend(createMoveControls(fixed=False))
    #server.insert(int_marker, processFeedback)
    
    # create an interactive marker for joint5 
    currentJoint5.position.x = 0
    currentJoint5.position.y = 0
    currentJoint5.position.z = 0.64
    qjoint = tf.transformations.quaternion_from_euler(-math.pi, math.pi/2, math.pi)
    currentJoint5.orientation.x = qjoint[0]
    currentJoint5.orientation.y = qjoint[1]
    currentJoint5.orientation.z = qjoint[2]
    currentJoint5.orientation.w = qjoint[3]
    p = currentJoint5.position
    o = currentJoint5.orientation
    int_marker = createInteractiveMarker('joint5',p.x, p.y, p.z, o.x, o.y, o.z, o.w, frame_id='link_4x')
    int_marker.controls.extend(createMoveControls(fixed=False))
    #server.insert(int_marker, processFeedback)
    
    # create an interactive marker for joint6 
    currentJoint6.position.x = -0.2
    currentJoint6.position.y = 0
    currentJoint6.position.z = 0
    qjoint = tf.transformations.quaternion_from_euler(math.pi/2, 0, -math.pi/2)
    currentJoint6.orientation.x = qjoint[0]
    currentJoint6.orientation.y = qjoint[1]
    currentJoint6.orientation.z = qjoint[2]
    currentJoint6.orientation.w = qjoint[3]
    p = currentJoint6.position
    o = currentJoint6.orientation
    int_marker = createInteractiveMarker('joint6',p.x, p.y, p.z, o.x, o.y, o.z, o.w, frame_id='link_5x')
    int_marker.controls.extend(createMoveControls(fixed=False))
    #server.insert(int_marker, processFeedback)
    rospy.Timer(rospy.Duration(0.01), frameCallbackLinks)


    currentJoint_tag.position.x = 0
    currentJoint_tag.position.y = 0.7
    currentJoint_tag.position.z = -0.2145
    qjoint = tf.transformations.quaternion_from_euler(0, 0, 0)
    currentJoint_tag.orientation.x = qjoint[0]
    currentJoint_tag.orientation.y = qjoint[1]
    currentJoint_tag.orientation.z = qjoint[2]
    currentJoint_tag.orientation.w = qjoint[3]
    p = currentJoint_tag.position
    o = currentJoint_tag.orientation
    int_marker = createInteractiveMarker('joint_tag',p.x, p.y, p.z, o.x, o.y, o.z, o.w, frame_id='link_2')
    cubemarker = createCubeMarker(offset=(p.x, p.y, p.z), rgba=(1,0,1,0.5),
                          orientation=(o.x, o.y, o.z, o.w),
                          scale=(0.01,0.01,0.01))
    #int_marker.controls.append(cubemarker)
    int_marker.controls.extend(createMoveControls(fixed=False))
    server.insert(int_marker, processFeedback)
    rospy.Timer(rospy.Duration(0.01), frameCallbackLinks)


    server.applyChanges()
    
    
    rospy.spin()

