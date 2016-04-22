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
from ik.roshelper import rosposeTransform
from ik.roshelper import poseTransform
from interactive_markers.interactive_marker_server import *
global brKinect
global brShelf
    
from marker_helper import createMoveControls
from marker_helper import createMeshMarker
from marker_helper import createCubeMarker

import optparse
import math

def frameCallbackKinect( msg ):
    global brKinect
    global currentKinectPose
    global currentJoint_realsense
    global listener
    global currentJoint_realsense_link5
    global nkinect
    global vis_pub
    global opt
    global pyramid_marker
    time = rospy.Time.now()
    
    for i in range(nkinect):
        p = currentKinectPose[i].position
        o = currentKinectPose[i].orientation
            
        brKinect.sendTransform( (p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
        time, "kinect2_%d_rgb_optical_frame" % (i+1), "map" )
    
    p = currentJoint_realsense_link5.position
    o = currentJoint_realsense_link5.orientation
    brKinect.sendTransform( (p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
    time, "realsense_link", "link_5" )

    # if opt.useCamMarker or opt.useAllMarker:
        # vis_pub.publish(pyramid_marker)


def frameCallbackShelf( msg ):
    global brShelf
    global currentShelfPose
    time = rospy.Time.now()
    
    p = currentShelfPose.position
    o = currentShelfPose.orientation
    
    brShelf.sendTransform( (p.x, p.y, p.z), (o.x, o.y, o.z, o.w),
    time, "shelf", "map")
    

def processFeedback(feedback):
    global currentExpoPose, currentCrayolaPose, currentKinectPose, currentShelfPose, currentJoint_realsense, currentJoint_realsense_link5
    global listener
    import rospy
    p = feedback.pose.position
    o = feedback.pose.orientation
    print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)+", "+str(o.x)+", "+str(o.y)+", "+str(o.z)+ ", "+str(o.w)
    
    if feedback.marker_name == 'ExpoEraser':
        currentExpoPose = feedback.pose
    if feedback.marker_name == 'Crayola':
        currentCrayolaPose = feedback.pose
    if feedback.marker_name == 'Kinect1':   # todo : improve to nkinect
        currentKinectPose[0] = feedback.pose
    if feedback.marker_name == 'Kinect2':
        currentKinectPose[1] = feedback.pose
    if feedback.marker_name == 'Shelf':
        currentShelfPose = feedback.pose
        p = currentShelfPose.position
        o = currentShelfPose.orientation
        # rospy.set_param('shelf_pose/x', p.x)
        # rospy.set_param('shelf_pose/y', p.y)
        # rospy.set_param('shelf_pose/z', p.z)
        # rospy.set_param('shelf_pose/qx', o.x)
        # rospy.set_param('shelf_pose/qy', o.y)
        # rospy.set_param('shelf_pose/qz', o.z)
        # rospy.set_param('shelf_pose/qw', o.w)
    if feedback.marker_name == 'joint_realsense':
        currentJoint_realsense = feedback.pose
        currentJoint_realsense_link5 = rosposeTransform(currentJoint_realsense, 'map', 'link_5', listener)
        p = currentJoint_realsense_link5.position
        o = currentJoint_realsense_link5.orientation
        print "currentJoint_realsense_link5 is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)+", "+str(o.x)+", "+str(o.y)+", "+str(o.z)+ ", "+str(o.w)


def createInteractiveMarker(name, x=0, y=0, z=0, ox=0, oy=0, oz=0, ow=1, frame_id="/map"):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = frame_id
    int_marker.name = name
    int_marker.scale = 0.3
    int_marker.description = name
    int_marker.pose.position.x = x
    int_marker.pose.position.y = y
    int_marker.pose.position.z = z
    int_marker.pose.orientation.x = ox
    int_marker.pose.orientation.y = oy
    int_marker.pose.orientation.z = oz
    int_marker.pose.orientation.w = ow
    
    global currentExpoPose, currentCrayolaPose
    if int_marker.name == 'ExpoEraser':
        currentExpoPose = int_marker.pose
    if int_marker.name == 'Crayola':
        currentCrayolaPose = int_marker.pose
    
    return int_marker

def getPose(req):
    global currentExpoPose, currentCrayolaPose
        
    print "Returning poses"
    ret = ObjectPoseList()
    
    objpose = ObjectPose()
    objpose.name = 'expo_dry_erase_board_eraser'
    objpose.pose = currentExpoPose
    ret.object_list.append(objpose)
    
    objpose = ObjectPose()
    objpose.name = 'crayola_64_ct'
    objpose.pose = currentCrayolaPose
    ret.object_list.append(objpose)
    
    return GetPoseResponse(ret)

def startPoseService():
    s = rospy.Service('pose_service', GetPose, getPose)

def list2pt(pointlist):
    p = Point()
    p.x = pointlist[0]
    p.y = pointlist[1]
    p.z = pointlist[2]
    return p

def createViewMarker(rgba=(1,0.5,0.5,0.5), offset=(0,0,0), orientation=(0,0,0,1), frame_id='map'):
    vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.sleep(0.1)
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = marker.TRIANGLE_LIST
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    
    d = 0.2
    origin = [0,0,0]
    halfThetaU = 0.783984717 /2
    halfThetaV = 1.334023 /2
    points = []
    points.append([math.tan(halfThetaV)*d, math.tan(halfThetaU)*d, d])
    points.append([-math.tan(halfThetaV)*d, math.tan(halfThetaU)*d, d])
    points.append([math.tan(halfThetaV)*d, -math.tan(halfThetaU)*d, d])
    points.append([-math.tan(halfThetaV)*d, -math.tan(halfThetaU)*d, d])
    #print points
        
    color = ColorRGBA()
    marker.points.append(list2pt(origin));    marker.points.append(list2pt(points[0]));    marker.points.append(list2pt(points[1]))
    #marker.colors.append(color)
    
    marker.points.append(list2pt(origin));    marker.points.append(list2pt(points[1]));    marker.points.append(list2pt(points[2]))
    #marker.colors.append(color)
    
    marker.points.append(list2pt(origin));    marker.points.append(list2pt(points[2]));    marker.points.append(list2pt(points[3]))
    #marker.colors.append(color)
    
    marker.points.append(list2pt(origin));    marker.points.append(list2pt(points[3]));    marker.points.append(list2pt(points[0]))
    #marker.colors.append(color)
    
    marker.points.append(list2pt(points[0]));    marker.points.append(list2pt(points[1]));    marker.points.append(list2pt(points[2]))
    #marker.colors.append(color)
    
    marker.points.append(list2pt(points[0]));    marker.points.append(list2pt(points[3]));    marker.points.append(list2pt(points[2]))
    #marker.colors.append(color)
        
    marker.pose.orientation.x = orientation[0]
    marker.pose.orientation.y = orientation[1]
    marker.pose.orientation.z = orientation[2]
    marker.pose.orientation.w = orientation[3]
    marker.pose.position.x = offset[0]
    marker.pose.position.y = offset[1]
    marker.pose.position.z = offset[2]
    
    # for i in range(10):
        # vis_pub.publish(marker)
        # rospy.sleep(0.1)
    
    obj_control = InteractiveMarkerControl()
    obj_control.always_visible = True
    obj_control.markers.append( marker )
        
    return obj_control


def main(argv=None):
    if argv is None:
        argv = sys.argv
    
    parser = optparse.OptionParser()
    
    #import socket
    #useObject = not (socket.gethostname() == 'mcube-002' or socket.gethostname() == 'mcube-003')
    
    parser.add_option('-o', '--obj', action="store_true", dest='useObject', help='To use manual object fitting', 
                      default=False)
                      
    parser.add_option('-c', '--cammarker', action="store_true", dest='useCamMarker', help='To use manual camera fitting or not', 
                      default=False)
                      
    parser.add_option('-s', '--shelfmarker', action="store_true", dest='useShelfMarker', help='To use manual shelf fitting or not', 
                      default=False)
                      
    parser.add_option('-a', '--all', action="store_true", dest='useAllMarker', help='To use manual fitting or not', 
                      default=False)
    
    
    rospy.init_node("manual_fit_server")
    
    global currentExpoPose
    global currentCrayolaPose
    global currentKinectPose
    global currentShelfPose
    global currentJoint_realsense
    global currentJoint_realsense_link5
    global brKinect
    global brShelf
    global nkinect
    global vis_pub
    global opt
    global pyramid_marker
    
    (opt, args) = parser.parse_args()
    currentExpoPose = Pose()
    currentCrayolaPose = Pose()
    currentKinectPose = [Pose(),Pose()]
    currentShelfPose = Pose()
    currentJoint_realsense = Pose()
    brKinect = TransformBroadcaster()
    brShelf = TransformBroadcaster()
    
    global listener
    listener = tf.TransformListener()
    rospy.sleep(1)
    # create an interactive marker server on the topic namespace simple_marker
    server = InteractiveMarkerServer("affordance_marker")
    
    # on mcube-002 don't use object
    
    if opt.useObject or opt.useAllMarker:
        # create an interactive marker for Expo
        pose = [1.58220779896, 0.296458333731, 1.12021064758, -0.00197346811183, -0.738883018494, 0.00179956667125, 0.673828423023]
        int_marker = createInteractiveMarker('ExpoEraser', *pose)
        
        meshmarker = createMeshMarker('package://apc_config/models/object_meshes/expo_dry_erase_board_eraser.stl', 
                                  offset=tuple(pose[0:3]), rgba=(0,1,0,1),
                                  orientation=tuple(pose[3:7]))
        int_marker.controls.append(meshmarker)
        int_marker.controls.extend(createMoveControls())
        server.insert(int_marker, processFeedback)
        
        # create an interactive marker for Crayola
        pose = [1.63922035694, -0.273455768824, 1.12480783463, -0.0384246744215, 0.697018921375, -0.715074837208, 0.0368278548121]
        int_marker = createInteractiveMarker('Crayola', *pose)
        meshmarker = createMeshMarker('package://apc_config/models/object_meshes/crayola_64_ct.stl', 
                                  offset=tuple(pose[0:3]), rgba=(1,0,0,1),
                                  orientation=tuple(pose[3:7]))
        int_marker.controls.append(meshmarker)
        int_marker.controls.extend(createMoveControls())
        server.insert(int_marker, processFeedback)

    # create an interactive marker for Shelf
    currentShelfPose.position.x = rospy.get_param('shelf_pose/x')
    currentShelfPose.position.y = rospy.get_param('shelf_pose/y')
    currentShelfPose.position.z = rospy.get_param('shelf_pose/z')
    currentShelfPose.orientation.x = rospy.get_param('shelf_pose/qx')
    currentShelfPose.orientation.y = rospy.get_param('shelf_pose/qy')
    currentShelfPose.orientation.z = rospy.get_param('shelf_pose/qz')
    currentShelfPose.orientation.w = rospy.get_param('shelf_pose/qw')
    
    int_marker = createInteractiveMarker('Shelf', currentShelfPose.position.x, currentShelfPose.position.y, currentShelfPose.position.z,
             currentShelfPose.orientation.x, currentShelfPose.orientation.y, currentShelfPose.orientation.z, currentShelfPose.orientation.w)
    meshmarker = createMeshMarker('package://apc_config/models/kiva_pod/meshes/pod_lowres.stl', 
                              offset=(currentShelfPose.position.x, currentShelfPose.position.y, currentShelfPose.position.z), rgba=(0,0,1,0.5),
                              orientation=(currentShelfPose.orientation.x, currentShelfPose.orientation.y, currentShelfPose.orientation.z, currentShelfPose.orientation.w))
    int_marker.controls.append(meshmarker)
    if opt.useShelfMarker or opt.useAllMarker:
        int_marker.controls.extend(createMoveControls())
    server.insert(int_marker, processFeedback)
    rospy.Timer(rospy.Duration(0.01), frameCallbackShelf)
    
    # create an interactive marker for Kinects
    nkinect = 2
    
    # old manual fit kinect pose
    #pose = [[0.305660784245, -0.883868515491, 1.66722476482, -0.244159132242, 0.827043175697, 0.0950953885913, 0.497335940599],
    #        [0.367319256067, 0.855090618134, 1.78848266602, 0.800817668438, -0.247087046504, 0.528468191624, 0.135500892997]]
            
    #pose = [[0.2643078824103202, -0.8475994497881857, 1.5983246933067445, -0.23088847539406132, 0.822983396811164, 0.08995383249481428, 0.511169994763495],
            #[0.3659496377911322, 0.8087564208783671, 1.779781933883937, 0.8065800933146582, -0.24219685501387836, 0.5258798075015827, 0.11924245508274978]]
            
    # pose  = [[0.2615747933866079, -0.847265732303331, 1.5924661390467871, -0.22946772410305405, 0.8209085501266424, 0.08173898334984725, 0.5164978476596324],
            # [0.35734815800402436, 0.809898201970986, 1.7640327819976598, 0.8036871694243676, -0.2417794937981231, 0.5299934028148222, 0.12138698747903716]]
    pose  = [[0.254395043470156, -0.849313121283084, 1.5914400427256015, -0.2499411509851439, 0.8071675924380618, 0.11813937247844475, 0.5215836382491776],
            [0.3581305540948658, 0.8062610720617674, 1.7711358584054768, 0.8090512113331657, -0.2868798404832606, 0.494998046288383, 0.13458167753046513]]

    for i in range(nkinect):
        currentKinectPose[i] = poselist2pose( pose[i] )
        
        if opt.useCamMarker or opt.useAllMarker:
            int_marker = createInteractiveMarker('Kinect%d' % (i+1), *pose[i])
            int_marker.controls.extend(createMoveControls(fixed=True))
            server.insert(int_marker, processFeedback)

    #pose = [-0.0246461518109, -0.000698581337929, 0.0109369456768, 0.706081748009, 0.706393122673, -0.0283508431166, 0.04066388309]
    #pose = [-0.030973277986, 0.0219592899084, -0.0248876661062, 0.702517032623, 0.711110830307, 0.0224311985075, 0.0169796515256]
    #pose = [-0.0141396149993, 0.0219592899084, -0.0248876661062, 0.70285320282, 0.71131336689, 0.00553341582417, -0.000123182311654]
    #pose = [-0.0186747914648, 0.00628428290067, -0.0175085197216, 0.702853216106, 0.711313378682, 0.00553342037479, -0.0001231898954]
    #pose = [-0.0191506993199, 0.0277181266145, -0.0117737966167, 0.70285321606, 0.711313378699, 0.00553342388632, -0.000123194876927]
    pose = [-0.023043141477, 0.0275422775115, -0.0114395739518, 0.70285321606, 0.711313378699, 0.00553342388632, -0.000123194876927]
    pose_world = poseTransform(pose, 'link_5', 'map', listener)
    currentJoint_realsense = poselist2pose( pose_world )
    currentJoint_realsense_link5 = poselist2pose( pose )

    if opt.useCamMarker or opt.useAllMarker:
        int_marker = createInteractiveMarker('joint_realsense', *pose_world, frame_id='map')
        # cubemarker = createCubeMarker(offset=tuple(pose[0:3]), rgba=(1,0,1,0.5),
                              # orientation=tuple(pose[3:7]),
                              # scale=(0.01,0.01,0.01))
        #int_marker.controls.append(cubemarker)
        viewMarker = createViewMarker(frame_id='realsense_depth_optical_frame')
        #points = [[0.15748027363422223, 0.08267716536239379, 0.2], [-0.15748027363422223, 0.08267716536239379, 0.2], [0.15748027363422223, -0.08267716536239379, 0.2], [-0.15748027363422223, -0.08267716536239379, 0.2] ]
        # meshmarker = createMeshMarker('package://apc_config/models/object_meshes/pyramid.stl', 
                      # offset=tuple([0,0,0]), rgba=(1,1,0,0.1),
                      # orientation=tuple([0,0,0,1]), frame_id='realsense_depth_optical_frame', scale = 0.001)
        # int_marker.controls.append(meshmarker)
        # 
        # meshmarker = createMeshMarker('package://apc_config/models/object_meshes/pyramid.stl', 
                      # offset=tuple([0,0,0]), rgba=(0,1,0,0.5),
                      # orientation=tuple([0,0,0,1]), frame_id='realsense_depth_optical_frame', scale = 0.002)
        # int_marker.controls.append(meshmarker)
        
        
        int_marker.controls.extend(createMoveControls(fixed=True))
        server.insert(int_marker, processFeedback)
        
        vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)

        pyramid_marker = createMeshMarker('package://apc_config/models/object_meshes/pyramid.stl', 
                      offset=tuple([0,0,0]), rgba=(1,1,0,0.1),
                      orientation=tuple([0,0,0,1]), frame_id='realsense_depth_optical_frame', scale = 0.002, scales = [2, 2,1]).markers[0]
    
    rospy.Timer(rospy.Duration(0.01), frameCallbackKinect)

    server.applyChanges()
    
    # ros services
    startPoseService()
    
    rospy.spin()
    

if __name__=="__main__":
    sys.exit(main())

