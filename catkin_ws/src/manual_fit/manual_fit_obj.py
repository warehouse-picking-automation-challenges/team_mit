#!/usr/bin/env python

import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math

topic = 'visualization_marker'
publisher = rospy.Publisher(topic, Marker)

rospy.init_node('register')

markerArray = MarkerArray()

count = 0
MARKERS_MAX = 100

while not rospy.is_shutdown():

   marker = Marker()
   marker.mesh_resource = "package://apc_config/models/object_meshes/expo_dry_erase_board_eraser.stl";
   marker.header.frame_id = "/world"
   marker.type = marker.MESH_RESOURCE
   marker.scale.x = 1
   marker.scale.y = 1
   marker.scale.z = 1
   marker.color.a = 1.0
   marker.color.r = 0.0
   marker.color.g = 1.0
   marker.color.b = 0.0
   marker.pose.orientation.w = 1.0
   marker.pose.position.x = 0
   marker.pose.position.y = 0
   marker.pose.position.z = 1


   publisher.publish(marker)
   rospy.sleep(0.01)
