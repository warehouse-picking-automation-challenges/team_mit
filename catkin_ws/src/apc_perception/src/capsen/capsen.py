#!/usr/bin/python

import sensor_msgs.msg
import roslib; roslib.load_manifest("capsen_vision")
from capsen_vision.srv import *
import rospy
from getpc.srv import *
import sys
import time
import tf
import tf.transformations as tfm
from ik.helper import get_obj_capsentf
from ik.roshelper import pubFrame
import numpy as np
import geometry_msgs.msg
from ik.helper import Timer
from ik.roshelper import pose2list
from ik.roshelper import poselist2pose
from ik.helper import graspGripper
from ik.helper import pause
from ik.helper import get_bin_cnstr
from ik.helper import matrix_from_xyzquat
from ik.helper import matrix_from_xyzquat
from ik.roshelper import poseTransform
from ik.roshelper import lookupTransform
from ik.helper import transformBack
from capsen_vision.msg import ObjectConstraint
import pdb
import traceback
import copy
import random
from visualization_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
import json


toHack = True
haveDelay = False

def get_filtered_pointcloud(obj_ids, bin_num, kinect_num):
    global _pointcloud2_service_srv
    with Timer('pointcloud2_service'):
        service_name = '/getpc_%d/getpc/get_filtered_pointcloud2_service' % kinect_num
        req = GetObjectPointCloud2Request()
        req.bin_num = bin_num
        req.obj_id = obj_ids[0]   # peterkty: need to pass in a list
        print '\tWaiting for service up: ', service_name
        rospy.wait_for_service(service_name)
        try:
            print '\tCalling service:', service_name
            response = _pointcloud2_service_srv[kinect_num-1](req)
            return response.pc2, response.foreground_mask
        except:
            print '\tCalling service:', service_name, 'failed'
            print '\tencounters errors:', traceback.format_exc()
            print '\tDid you call capsen.capsen.init()? Is camera connection good?'
            return None, None

bin_cnstr = get_bin_cnstr()

def inside_bin(point, bin_num):
    cnstr = bin_cnstr[bin_num]
    
    # return (
        # x > cnstr[0]+0.02 && x < cnstr[1]-0.02 &&
        # y > cnstr[2]+0.1 && y < cnstr[3]-0.01 &&
        # z > cnstr[4]+0.00 && z < cnstr[5]-0.02/* && z < cnstr[4]+obj_max_height*/);
        
    if  point[0] > cnstr[0]+0.015 and point[0] < cnstr[1]-0.015 and \
        point[1] > cnstr[2]+0.1 and point[1] < cnstr[3]-0.01 and \
        point[2] > cnstr[4]-0.02 and point[2] < cnstr[5]-0.02:
        return True
        
    # #todo: make the numbers out of python code
    return False

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def _detectOneObject(obj_id, bin_num, kinect_num):
    global _detect_one_object_srv
    global br
    
    print 'In', bcolors.WARNING, '_detectOneObject', bcolors.ENDC, 'obj_ids:', obj_id, 'bin_num:', bin_num
    # filter the point cloud
    pc, foreground_mask = get_filtered_pointcloud([obj_id], bin_num, kinect_num)
    if pc is None:
        return (None, None)
    
    # string[] model_names
    # sensor_msgs/PointCloud2 cloud   # Note: this must be an organized point cloud (e.g., 640x480)
    # bool[] foreground_mask
    # bool find_exact_object_list     # Set to true if you only want to consider scene hypotheses that contain exactly the objects in 'model_names'
    # ObjectConstraint[] constraints  # These apply to all objects
    # ---
    # SceneHypothesis[] detections
    
    
    # 2. prepare constraints
    
    bin_cnstr = get_bin_cnstr()[bin_num] # a list of right \ # left \ # back \  # front  \ # bottom \ # top
    ccnstr = []
    
    tol = 0.9  # larger is more strict
    (trans,rot) = lookupTransform(pc.header.frame_id, '/shelf', _tflistener)
    # 2.1 right
    ccnstr.append( createCapsenConstraint(ObjectConstraint.HALF_SPACE, transformPlane([1,0,0], [bin_cnstr[0],0,0], trans,rot, pc.header.frame_id), tol, bin_num) )
    # 2.2 left
    ccnstr.append( createCapsenConstraint(ObjectConstraint.HALF_SPACE, transformPlane([-1,0,0], [bin_cnstr[1],0,0], trans,rot, pc.header.frame_id), tol, bin_num) )
    # 2.3 back
    ccnstr.append( createCapsenConstraint(ObjectConstraint.HALF_SPACE, transformPlane([0,1,0], [0,bin_cnstr[2],0], trans,rot, pc.header.frame_id), tol, bin_num) )
    # 2.4 front
    ccnstr.append( createCapsenConstraint(ObjectConstraint.HALF_SPACE, transformPlane([0,-1,0], [0,bin_cnstr[3],0], trans,rot, pc.header.frame_id), tol, bin_num) )
    # 2.5 floor
    ccnstr.append( createCapsenConstraint(ObjectConstraint.HALF_SPACE, transformPlane([0,0,1], [0,0,bin_cnstr[4]], trans,rot, pc.header.frame_id), tol, bin_num) )
    # 2.6 top
    ccnstr.append( createCapsenConstraint(ObjectConstraint.HALF_SPACE, transformPlane([0,0,-1], [0,0,bin_cnstr[5]], trans,rot, pc.header.frame_id), tol, bin_num) )
    # 2.7 on floor
    floor_thick = 0.03
    ccnstr.append( createCapsenConstraint(ObjectConstraint.SUPPORTING_PLANE, transformPlane([0,0,1], [0,0, bin_cnstr[4]-floor_thick/2], trans,rot, pc.header.frame_id), tol, bin_num) )
    # string model_name
    # sensor_msgs/PointCloud2 cloud   # Note: this must be an organized point cloud (e.g., 640x480)
    # bool[] foreground_mask
    # ObjectConstraint[] constraints
    # geometry_msgs/Pose true_pose    # for testing
    # ---
    # 
    # ObjectHypothesis[] detections

    with Timer('detect_one_object'):
        # detect using capsen
        service_name = '/detection_service/detect_one_object'
        req = DetectOneObjectRequest()
        req.model_name = obj_id
        req.cloud = pc
        req.constraints = ccnstr
        req.foreground_mask = foreground_mask
        #req.foreground_mask = [True for i in xrange(req.cloud.height*req.cloud.width)]
        
        print 'Waiting for service up: ', service_name
        rospy.wait_for_service(service_name)
        try:
            print 'Calling service:', service_name
            ret = _detect_one_object_srv(req)
            # ret.detections is a list of capsen_vision/ObjectHypothesis
                # string name
                # geometry_msgs/Pose pose
                # float32 score
                # float32[] score_components
            if len(ret.detections)>0:
                print len(ret.detections), 'ObjectHypothesis returned, max score', ret.detections[0].score
                for i in range(len(ret.detections)):
                    poselist_capsen_world = poseTransform(pose2list(ret.detections[i].pose), pc.header.frame_id, 'map', _tflistener)
                    
                    cap_T_our = get_obj_capsentf(obj_id) # x,y,z,qx,qy,qz,qw
                    poselist_world = transformBack(cap_T_our, poselist_capsen_world)    # transform to our desired pose
                    
                    # check whether inside bin
                    poselist_shelf = poseTransform(poselist_world, 'map', 'shelf', _tflistener)
                    if inside_bin(poselist_shelf[0:3], bin_num):
                        #pubFrame(br, poselist_world, 'obj', 'map')
                        return (poselist_world, ret.detections[i].score)
                    else:
                        print 'reject hypo', i, 'because it is outside the target bin'
                print 'No ObjectHypothesis satisfy hard bin constraint'
                return (None, None)
            else:
                print 'No ObjectHypothesis returned'
                return (None, None)
        except:
            print 'Calling service:', service_name, 'failed'
            print 'encounters errors:', traceback.format_exc()
            return (None, None)


# mode = 0: use 2 kinects only
# mode = 1: use realsense only
def detectOneObjectWithoutBinContents(target_obj_id, bin_num, mode = 0):
    nretry = 3
    maxPose = None
    maxScore = -100   # some very small number
    if mode == 0:
        for j in range(1,3):  # loop over 2 cams
            for i in range(nretry):
                retPose, retScore = _detectOneObject(target_obj_id, bin_num, j)
                if retPose is not None and retScore > maxScore:
                    maxPose = retPose;  maxScore = retScore;
                    pubFrame(br, maxPose, 'obj_final', 'map')
        

    elif mode == 1:  # realsense
        for i in range(nretry*2):
            retPose, retScore = _detectOneObject(target_obj_id, bin_num, 3)
            if retPose is not None and retScore > maxScore:
                maxPose = retPose;  maxScore = retScore;
                pubFrame(br, maxPose, 'obj_final', 'map')
    
    else:
        print 'Mode incorrect!'
        return None
    
    return maxPose


def createCapsenConstraint(cnstr_type, params, tolerance, bin_num):
    constraint = ObjectConstraint(); 
    constraint.type = cnstr_type
    constraint.params = params
    return constraint

def transformPlane(params, pt, trans, rot, target_frame_id):
    # params = [a,b,c,d]  -> ax+by+cz>=d,   we ignore input d
    # pt is a point on the plane [x,y,z]  (a list) 
    global br
    
    rotMat = tfm.quaternion_matrix(rot)
    target_T_source = matrix_from_xyzquat(trans, rot)
    normal = params[0:3]
    normal_target = np.dot(rotMat, np.array(normal + [0])) [0:3]
    pt_target = np.dot(target_T_source, np.array(pt + [1])) [0:3]
    d_target = np.dot(normal_target, pt_target)
    ret = normal_target.tolist() + [d_target]
    #pubFrame(br, pt_target.tolist() + [0,0,0,1], 'bound:%f' % d_target, target_frame_id)
    #print ret
    #pause()
    return ret

def transformObjectsFromCapsenToDesiredFrame(scene, scene_frame_id):
    global _tflistener
    global br
    newscene = copy.deepcopy(scene)
    for i in range(len(scene.objects)):
        poselist_capsen_world = poseTransform(pose2list(scene.objects[i].pose), scene_frame_id, 'map', _tflistener)
        cap_T_our = get_obj_capsentf(scene.objects[i].name) # x,y,z,qx,qy,qz,qw
        poselist_world = transformBack(cap_T_our, poselist_capsen_world)  
        newscene.objects[i].pose = poselist2pose(poselist_world)
        
        #pubFrame(br, poselist_world, 'obj_%s' % scene.objects[i].name, 'map')
    return newscene

def allObjectsInsideBin(scene, bin_num):
    global _tflistener
    for i in range(len(scene.objects)):
        poselist_shelf = poseTransform(pose2list(scene.objects[i].pose), 'map', 'shelf', _tflistener)
        if not inside_bin(poselist_shelf[0:3], bin_num):
            return False
        
    return True

def visualizeConstraint(cnstr, frame_id):
    params = cnstr.params
    #constraint.type = cnstr_type
    pts = []
    for x in np.linspace(-1, 1, num=100):
        for y in np.linspace(-1, 1, num=100):
            for z in np.linspace(0, 1, num=100):
                val = params[0]*x + params[1]*y + params[2]*z 
                if val >= params[3]:
                    pts.append([x,y,z])
                    
    showPointMarker(pts, frame_id)
    
def showPointMarker(points, frame_id, offset=(0,0,0), orientation=(0,0,0,1)):
    vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rospy.sleep(0.1)
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.type = marker.POINTS
    marker.scale.x = 0.003
    marker.scale.y = 0.003
    marker.scale.z = 0.003
    
    n = len(points)//3
    for pt in points:
        p = Point()
        p.x = pt[0]
        p.y = pt[1]
        p.z = pt[2]
        marker.points.append(p)
        
        p = ColorRGBA()
        p.r = 0
        p.g = 0
        p.b = 1
        p.a = 1
        marker.colors.append(p)
        
    marker.pose.orientation.x = orientation[0]
    marker.pose.orientation.y = orientation[1]
    marker.pose.orientation.z = orientation[2]
    marker.pose.orientation.w = orientation[3]
    marker.pose.position.x = offset[0]
    marker.pose.position.y = offset[1]
    marker.pose.position.z = offset[2]
    
    vis_pub.publish(marker)
    rospy.sleep(0.1)
    
    
def allFalse(foreground_mask, cnt):
    thres = 200
    print '\tallFalse(): %d valid points, thres = %d' % (cnt, thres)
    if cnt < thres:
        return True
    return False

def subsample(foreground_mask, cnt):
    target_cnt = 5000
    if cnt > target_cnt:
        ratio = float(target_cnt) / cnt
        for i in range(len(foreground_mask)):
            foreground_mask[i] = foreground_mask[i] and random.uniform(0,1) < ratio
            
    
    print '\tsubsample(): After subsample: %d valid points' % sum(foreground_mask)
    return foreground_mask

def _detectObjects(obj_ids, bin_num, kinect_num):
    # return pose, retScore
    global _detect_objects_srv
    global br
    global _tflistener
    
    print 'In', '_detectObjects', 'obj_ids:', obj_ids, 'bin_num:', bin_num
    
    # 1. filter the point cloud
    pc, foreground_mask = get_filtered_pointcloud(obj_ids, bin_num, kinect_num)  # need to pass in list
    if pc is None or foreground_mask is None:
        return (None, None)
    
    
    
    # 2. prepare constraints
    
    bin_cnstr = get_bin_cnstr()[bin_num] # a list of right \ # left \ # back \  # front  \ # bottom \ # top
    ccnstr = []
    
    tol = 0.9  # larger is more strict
    (trans,rot) = lookupTransform(pc.header.frame_id, '/shelf', _tflistener)
    # 2.1 right
    ccnstr.append( createCapsenConstraint(ObjectConstraint.HALF_SPACE, transformPlane([1,0,0], [bin_cnstr[0],0,0], trans,rot, pc.header.frame_id), tol, bin_num) )
    # 2.2 left
    ccnstr.append( createCapsenConstraint(ObjectConstraint.HALF_SPACE, transformPlane([-1,0,0], [bin_cnstr[1],0,0], trans,rot, pc.header.frame_id), tol, bin_num) )
    # 2.3 back
    ccnstr.append( createCapsenConstraint(ObjectConstraint.HALF_SPACE, transformPlane([0,1,0], [0,bin_cnstr[2],0], trans,rot, pc.header.frame_id), tol, bin_num) )
    # 2.4 front
    ccnstr.append( createCapsenConstraint(ObjectConstraint.HALF_SPACE, transformPlane([0,-1,0], [0,bin_cnstr[3],0], trans,rot, pc.header.frame_id), tol, bin_num) )
    # 2.5 floor
    ccnstr.append( createCapsenConstraint(ObjectConstraint.HALF_SPACE, transformPlane([0,0,1], [0,0,bin_cnstr[4]], trans,rot, pc.header.frame_id), tol, bin_num) )
    # 2.6 top
    ccnstr.append( createCapsenConstraint(ObjectConstraint.HALF_SPACE, transformPlane([0,0,-1], [0,0,bin_cnstr[5]], trans,rot, pc.header.frame_id), tol, bin_num) )
    # 2.7 on floor
    floor_thick = 0.03
    ccnstr.append( createCapsenConstraint(ObjectConstraint.SUPPORTING_PLANE, transformPlane([0,0,1], [0,0, bin_cnstr[4]+floor_thick*2], trans,rot, pc.header.frame_id), tol, bin_num) )
    #visualizeConstraint(ccnstr[6], pc.header.frame_id)
    #pause()
    
    # 3. detect using capsen
    with Timer('detect_objects'):
        service_name = '/detection_service/detect_objects'
        req = DetectObjectsRequest()
        req.model_names = obj_ids
        req.constraints = ccnstr
        req.cloud = pc
        req.foreground_mask = foreground_mask
        
        sum_pt = sum(foreground_mask)
        if allFalse(foreground_mask, sum_pt):
            return (None, None)
        foreground_mask = subsample(foreground_mask, sum_pt)
        
        # outputfile = '/tmp/foreground_mask'
        # with open(outputfile, 'w') as outfile:
            # json.dump(foreground_mask, outfile)
        # pause()
        #req.foreground_mask = [True for i in xrange(req.cloud.height*req.cloud.width)]  # hack
        req.find_exact_object_list = True
        
        print '\tWaiting for service up: ', service_name
        rospy.wait_for_service(service_name)
        
        #pdb.set_trace()
        try:
            print '\tCalling service:', service_name
            ret = _detect_objects_srv(req)
            # ret.detections is a list of capsen_vision/SceneHypothesis
            # [capsen_vision/SceneHypothesis]:
                # std_msgs/Header header
                  # uint32 seq
                  # time stamp
                  # string frame_id
                # capsen_vision/ObjectHypothesis[] objects
                  # string name
                  # geometry_msgs/Pose pose
                    # geometry_msgs/Point position
                      # float64 x
                      # float64 y
                      # float64 z
                    # geometry_msgs/Quaternion orientation
                      # float64 x
                      # float64 y
                      # float64 z
                      # float64 w
                  # float32 score
                  # float32[] score_components
                  # float32[2] errors
                # float32 score

            if len(ret.detections)>0:
                print '\t', len(ret.detections), 'SceneHypothesis returned, max score', ret.detections[0].score
                #print ret.detections
                for i in range(len(ret.detections)):
                    scene = ret.detections[i]
                    nobj = len(scene.objects)
                    
                    scene_desired = transformObjectsFromCapsenToDesiredFrame(scene, pc.header.frame_id)
                    if allObjectsInsideBin(scene_desired, bin_num):
                        return (scene_desired.objects, scene_desired.score)
                    #else:
                        #print 'reject scene hypo', i, 'because one object of it is outside the target bin'
                print '\tNo SceneHypothesis satisfy hard bin constraint'
                return (None, None)
            else:
                print '\tNo SceneHypothesis returned'
                return (None, None)
        except:
            print '\tCalling service:', service_name, 'failed'
            print '\tencounters errors:', traceback.format_exc()
            return (None, None)


def findTargetInd(Objects, target_obj_id):
    for i, obj in enumerate(Objects):
        if obj.name == target_obj_id:
            return i
    return None

def detectObjects(target_obj_id, obj_ids, bin_num, mode = 0):
    nretry = 3
    maxObjects = None
    maxScore = -100   # some very small number
    if mode == 0:
        for j in range(1,3):  # loop over 2 cams
            for i in range(nretry):
                retObjects, retScore = _detectObjects(obj_ids, bin_num, j)
                if retObjects is not None and retScore > maxScore:
                    target_ind = findTargetInd(retObjects, target_obj_id)
                    if target_ind is not None:
                        maxObjects = retObjects;  maxScore = retScore;
                        maxPose = pose2list(retObjects[target_ind].pose)
                        pubFrame(br, maxPose, 'obj_final', 'map')

    elif mode == 1:  # realsense
        for i in range(nretry):
            retObjects, retScore = _detectObjects(obj_ids, bin_num, 3)
            if retObjects is not None and retScore > maxScore:
                target_ind = findTargetInd(retObjects, target_obj_id)
                if target_ind is not None:
                    maxObjects = retObjects;  maxScore = retScore;
                    maxPose = pose2list(retObjects[target_ind].pose)
                    pubFrame(br, maxPose, 'obj_final', 'map')
    
    else:
        print 'Mode incorrect!'
        return (None, None)
    
    return (maxObjects, maxScore)

def randomPoseScore(bin_num, withScore):
    typical_poses = \
    [[1.58220350742, 0.287826299667, 1.12025654316, -0.00197346811183, -0.738883018494, 0.00179956667125, 0.673828423023],
    [1.58204042912, -0.0443051755428, 1.12202310562, -0.00197346811183, -0.738883018494, 0.00179956667125, 0.673828423023],
    [1.58190357685, -0.323061853647, 1.12350583076, -0.00197346811183, -0.738883018494, 0.00179956667125, 0.673828423023],
    [1.58220350742, 0.287826299667, 0.901469767094, -0.00197346811183, -0.738883018494, 0.00179956667125, 0.673828423023],
    [1.58204042912, -0.0443051755428, 0.9014697670942, -0.00197346811183, -0.738883018494, 0.00179956667125, 0.673828423023],
    [1.58190357685, -0.323061853647, 0.901469767094, -0.00197346811183, -0.738883018494, 0.00179956667125, 0.673828423023],
    [1.58220350742, 0.287826299667, 0.658816933632, -0.00197346811183, -0.738883018494, 0.00179956667125, 0.673828423023],
    [1.58204042912, -0.0443051755428, 0.658816933632, -0.00197346811183, -0.738883018494, 0.00179956667125, 0.673828423023],
    [1.58190357685, -0.323061853647, 0.658816933632, -0.00197346811183, -0.738883018494, 0.00179956667125, 0.673828423023],
    [1.58220350742, 0.287826299667, 0.434227764606, -0.00197346811183, -0.738883018494, 0.00179956667125, 0.673828423023],
    [1.58204042912, -0.0443051755428, 0.434227764606, -0.00197346811183, -0.738883018494, 0.00179956667125, 0.673828423023],
    [1.58190357685, -0.323061853647, 0.434227764606, -0.00197346811183, -0.738883018494, 0.00179956667125, 0.673828423023]]
    obj_pose = typical_poses[bin_num]
    obj_pose[0] += random.uniform(-0.1, 0.1)
    obj_pose[1] += random.uniform(-0.1, 0.1)
    obj_pose[2] += random.uniform(-0.1, 0.1)
    obj_pose[3:7] = tfm.random_quaternion(rand=None).tolist()
    thres = 0
    if withScore:
        if random.uniform(0,1) >= thres:
            return (obj_pose, random.uniform(0.1, 3))
        else:
            return None, None
    else:
        if random.uniform(0,1) >= thres:
            return obj_pose
        else:
            return None
        
def detectOneObject(target_obj_id, obj_ids, bin_num, mode = 0, withScore = False):
    # hack
    if toHack:
        # may want to have delay
        if haveDelay:
            nretry = 4
            timeforcapsen = 3.0
            time_ = nretry*timeforcapsen
            if mode == 0:
                time_ *= 2
            rospy.sleep(time_)
            print '[detectOneObject] simulated computation time %.2f sec' % time_
        return randomPoseScore(bin_num, withScore)
    
    retObjects, retScore = detectObjects(target_obj_id, obj_ids, bin_num, mode)
    # find the target object
    if retObjects is not None:
        for obj in retObjects:
            if obj.name == target_obj_id:
                if withScore:
                    return (pose2list(obj.pose), retScore)
                else:
                    return pose2list(obj.pose)
    
    if withScore:
        return (None, None)
    else:
        return None
    
initialized = False

def init():
    global _pointcloud2_service_srv
    global _detect_one_object_srv
    global _detect_objects_srv
    global _tflistener
    global br
    global initialized
    
    if initialized:   # already done
        return
    else:
        initialized = True
    
    _tflistener = tf.TransformListener()
    br = tf.TransformBroadcaster() # for visualizing the detected frame
    
    _pointcloud2_service_srv = []
    
    kinect_num = 1
    service_name = '/getpc_%d/getpc/get_filtered_pointcloud2_service' % kinect_num
    _pointcloud2_service_srv.append(rospy.ServiceProxy(service_name, GetObjectPointCloud2))
    
    kinect_num = 2
    service_name = '/getpc_%d/getpc/get_filtered_pointcloud2_service' % kinect_num
    _pointcloud2_service_srv.append(rospy.ServiceProxy(service_name, GetObjectPointCloud2))
    
    kinect_num = 3
    service_name = '/getpc_%d/getpc/get_filtered_pointcloud2_service' % kinect_num
    _pointcloud2_service_srv.append(rospy.ServiceProxy(service_name, GetObjectPointCloud2))
    
    _detect_one_object_srv= rospy.ServiceProxy('/detection_service/detect_one_object', DetectOneObject)
    
    _detect_objects_srv= rospy.ServiceProxy('/detection_service/detect_objects', DetectObjects)
    
    rospy.sleep(0.5)
    


def main(argv=None):
    global br
    if argv is None:
        argv = sys.argv
    
    rospy.init_node('capsen_test', anonymous=True)
    rospy.sleep(0.1)
    init()
    rospy.sleep(0.1)
    
    
    #retObjects, retScore = detectObjects('expo_dry_erase_board_eraser', ['expo_dry_erase_board_eraser', 'elmers_washable_no_run_school_glue'], bin_num = 0, mode = 0)
    # retObjects, retScore = detectObjects('expo_dry_erase_board_eraser', ['expo_dry_erase_board_eraser', 'elmers_washable_no_run_school_glue'], bin_num = 0, mode = 0)
    # pose = pose2list(retObjects[0].pose)
    # pubFrame(br, pose, 'obj', 'map')
    # print 'Objects', retObjects
    # pause()
    
    obj_list = ['mommys_helper_outlet_plugs', 
    'kong_duck_dog_toy',
    'first_years_take_and_toss_straw_cup',
    'champion_copper_plus_spark_plug',
    'mead_index_cards',
    'laugh_out_loud_joke_book',
    'highland_6539_self_stick_notes',
    'elmers_washable_no_run_school_glue',
    'stanley_66_052',
    'genuine_joe_plastic_stir_sticks',
    'safety_works_safety_glasses',
    'munchkin_white_hot_duck_bath_toy'
    # 'crayola_64_ct',
    # 'dr_browns_bottle_brush',
    # 'kyjen_squeakin_eggs_plush_puppies',
    # 'expo_dry_erase_board_eraser',
    # 'cheezit_big_original',
    # 'kong_air_dog_squeakair_tennis_ball',
    # 'safety_works_safety_glasses',
    # 'genuine_joe_plastic_stir_sticks'
    ]
    

    bin_contents_all = [
        [     "mommys_helper_outlet_plugs", "mark_twain_huckleberry_finn"      ],
        [        "feline_greenies_dental_treats", "kong_duck_dog_toy"        ],
        [        "first_years_take_and_toss_straw_cup","kong_sitting_frog_dog_toy"        ],
        [        "paper_mate_12_count_mirado_black_warrior", "champion_copper_plus_spark_plug"        ],
        [        "mead_index_cards", "sharpie_accent_tank_style_highlighters"        ],
        [        "mommys_helper_outlet_plugs", "laugh_out_loud_joke_book"        ],
        [        "kyjen_squeakin_eggs_plush_puppies", "highland_6539_self_stick_notes"        ],
        [        "elmers_washable_no_run_school_glue", "champion_copper_plus_spark_plug"        ],
        [        "crayola_64_ct", "stanley_66_052"        ],
        [        "genuine_joe_plastic_stir_sticks", "expo_dry_erase_board_eraser"        ],
        [        "safety_works_safety_glasses"        ],
        [        "kong_air_dog_squeakair_tennis_ball", "munchkin_white_hot_duck_bath_toy"        ]]
    
    #for i, obj_id in enumerate(obj_list):
    for i in range(5,12):
        pose = detectOneObject(obj_list[i], bin_contents_all[i], i)
        pubFrame(br, pose, 'obj_final', 'map')
        print 'Pose', pose
        pause()

    
    
if __name__ == "__main__":
    sys.exit(main())



