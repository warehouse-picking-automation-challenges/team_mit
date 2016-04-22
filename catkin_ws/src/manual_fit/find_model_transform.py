#!/usr/bin/env python
# find the transform from capsen's frame to our frame

import sys
import roslib; roslib.load_manifest("interactive_markers")
import rospy
import tf
import os
from manual_fit.srv import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
from pr_msgs.msg import *
from tf.broadcaster import TransformBroadcaster
from interactive_markers.interactive_marker_server import *
from ik.helper import get_obj_dim
from ik.helper import get_obj_capsentf
from marker_helper import createInteractiveMarker
from marker_helper import createMoveControls
from marker_helper import vizCubeMarker
from marker_helper import load_pcd
import pdb

def frameCallback( msg ):
    global br
    global currentObjPose
    time = rospy.Time.now()
    
    p = currentObjPose.position
    o = currentObjPose.orientation
        
    br.sendTransform( (p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
    time, "object", "map")
    
def createPointMarker(points, colors, offset=(0,0,0), orientation=(0,0,0,1), size=0.003): 
    #size: in meter
    marker = Marker()
    marker.header.frame_id = "/map"
    marker.type = marker.POINTS
    marker.scale.x = size
    marker.scale.y = size
    marker.scale.z = size
    
    for i in range(len(points)):
        p = Point()
        p.x = points[i][0]
        p.y = points[i][1]
        p.z = points[i][2]
        marker.points.append(p)
        
        p = ColorRGBA()
        p.r = colors[i][0]
        p.g = colors[i][1]
        p.b = colors[i][2]
        p.a = 1
        marker.colors.append(p)
        
    marker.pose.orientation.x = orientation[0]
    marker.pose.orientation.y = orientation[1]
    marker.pose.orientation.z = orientation[2]
    marker.pose.orientation.w = orientation[3]
    marker.pose.position.x = offset[0]
    marker.pose.position.y = offset[1]
    marker.pose.position.z = offset[2]
    
    obj_control = InteractiveMarkerControl()
    obj_control.always_visible = True
    obj_control.markers.append( marker )
        
    return obj_control

def processFeedback(feedback):
    #global currentExpoPose, currentCrayolaPose, currentKinectPose, currentShelfPose
    import rospy
    p = feedback.pose.position
    o = feedback.pose.orientation
    print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)+", "+str(o.x)+", "+str(o.y)+", "+str(o.z)+ ", "+str(o.w)
    currentObjPose.position = p
    currentObjPose.orientation = o
    

if __name__=="__main__":
    global br
    global currentObjPose
    currentObjPose = Pose()
    
    rospy.init_node("find_model_transform")
    br = TransformBroadcaster()
    
    obj_id = sys.argv[1]  # like oreo_mega_stuf
    filename = os.environ['APCDATA_BASE']+'/capsen_vision/models/'+obj_id+'/model.pcd'
    (points, colors) = load_pcd(filename)
    
    pose = get_obj_capsentf(obj_id) # x,y,z,qx,qy,qz,qw
    if len(pose) <7:
        pose = [0,0,0,0,0,0,1]
    
    #pdb.set_trace()
    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("affordance_marker")
    int_marker = createInteractiveMarker('Object',*pose)
    
    pointmarker = createPointMarker(colors=colors, points=points, offset=tuple(pose[0:3]), orientation=tuple(pose[3:7]))
    int_marker.controls.append(pointmarker)    
    
    int_marker.controls.extend(createMoveControls(fixed=True))
    
    #### create a static marker
    dim = get_obj_dim(obj_id)
    for i in range(5):
        vizCubeMarker(size=dim, rgba=(0,1,0,0.3))
    
    server.insert(int_marker, processFeedback)

    rospy.Timer(rospy.Duration(0.01), frameCallback)


    server.applyChanges()
    
    # ros services
    #startPoseService()
    
    rospy.spin()

























