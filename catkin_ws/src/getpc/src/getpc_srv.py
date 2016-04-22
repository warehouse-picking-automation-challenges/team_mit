#!/usr/bin/env python

# listen to point cloud and has a service that returns the point cloud(s) 
# given bin number and object id (e.g. oreo)

# roslaunch getpc getpc.launch
# roslaunch apc_config kinect2_bridge.launch
# rosservice call pointcloud_service

import roslib
import rospy
import tf
from getpc.srv import *
from sensor_msgs.msg import *
from sensor_msgs import point_cloud2
from multiprocessing import Process, Lock
from visualization_msgs.msg import *
from tf import TransformListener
from geometry_msgs.msg import Point32
import math
import ctypes
import pdb
import time
import copy


mutex = Lock()
#import sensor_msgs.point_cloud2

# the precise bounds gotten from the shelf mesh
# here only the top row is entered
# right \ # left \ # back \  # front  \ # bottom \ # top

bin_cnstr = [[0.1554, 0.42926, 0, 0.42, 1.52393, 1.79063],
             [-0.1494, 0.1494, 0, 0.42, 1.52393, 1.79063],
             [-0.42926, -0.1554, 0, 0.42, 1.52393, 1.79063],
             [0.1554, 0.42926, 0, 0.42, 1.29533, 1.52393],
             [-0.1494, 0.1494, 0, 0.42, 1.29533, 1.52393],
             [-0.42926, -0.1554, 0, 0.42, 1.29533, 1.52393],
             [0.1554, 0.42926, 0, 0.42, 1.06673, 1.29533],
             [-0.1494, 0.1494, 0, 0.42, 1.06673, 1.29533],
             [-0.42926, -0.1554, 0, 0.42, 1.06673, 1.29533],
             [0.1554, 0.42926, 0, 0.42, 0.800027, 1.06673],
             [-0.1494, 0.1494, 0, 0.42, 0.800027, 1.06673],
             [-0.42926, -0.1554, 0, 0.42, 0.800027, 1.06673]]
             

class Timer(object):
    def __init__(self, name=None):
        self.name = name

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        if self.name:
            print '[%s]' % self.name,
        print 'Elapsed: %s' % (time.time() - self.tstart)

def inside_bin(point, bin_num):
    
    cnstr = bin_cnstr[bin_num]
    if  point[0] > cnstr[0]+0.02 and point[0] < cnstr[1]-0.02 and \
        point[1] > cnstr[2]+0.1 and point[1] < cnstr[3]+0.02 and \
        point[2] > cnstr[4]+0.02 and point[2] < cnstr[5]-0.02:
        return True
        
    # #todo: make the numbers out of python code
    return False
    

# def inside_bin(point, bin_num):
    # 
    # cnstr = bin_cnstr[bin_num]
    # if  point[0] > cnstr[0]+0.02 and point[0] < cnstr[1]-0.02 and \
        # point[1] > cnstr[2]+0.45 and point[1] < cnstr[3]+0.80 and \
        # point[2] > cnstr[4]+0.02 and point[2] < cnstr[5]-0.02:
        # return True
        # 
    #todo: make the numbers out of python code
    # return False

def filterPointCloud(bin_num, pts, source_pc2_kinect_header, height=None, width=None, retPointCloud=True):
    # input height/width if you need filtered uv
    with Timer('b1'):
        global tfListener
        source_pc_kinect = PointCloud()
        source_pc_kinect.header = source_pc2_kinect_header
    with Timer('b2'):
        source_pc_kinect.points = pts
    
    # transform point cloud from kinect frmae to shelf frame
    with Timer('b3'):
        import tf.transformations as tfm
        import numpy as np
        (pos, ori) = tfListener.lookupTransform('/shelf', source_pc_kinect.header.frame_id, source_pc_kinect.header.stamp)
        
        points = []
        T = np.dot(tfm.compose_matrix(translate=pos) , tfm.quaternion_matrix(ori) )

        for pt in pts:
            #tmp = np.dot(T, np.array(pt+[1])).tolist()
            points.append( np.dot(T, np.array(pt+[1])).tolist() )
            
    pts = []
    cnt = 0
    
    with Timer('b4'):
        if height and width:
            filtered_uvs = []
            filtered_uvs_mask = [False for i in xrange(height*width)]
            for point in points:  # change source to source_pc_kinect
                if ~math.isnan(point[0]) and inside_bin(point, bin_num):
                    pts.append(point)   # can be removed becuase we only need mask
                    filtered_uvs_mask[cnt] = True
                    filtered_uvs.append([cnt//width, cnt%width])
                cnt += 1
        else:
            for point in points:  # change source to source_pc_kinect
                if inside_bin(point, bin_num):
                    pts.append(point)
    
    if retPointCloud:
        with Timer('b5'):
            filtered_pc_shelf = PointCloud()
            filtered_pc_shelf.header = source_pc2_kinect_header
            filtered_pc_shelf.header.frame_id = '/shelf'
            filtered_pc_shelf.points = [Point32(pt[0], pt[1], pt[2]) for pt in pts]
            
            # render filtered point cloud
            filtered_pc_map = tfListener.transformPointCloud('/map', filtered_pc_shelf)
            createPointsMarker(filtered_pc_map.points, marker_id=2, frame_id='/map', rgba=(0,1,0,1)) #points are in Point32
        
    
    with Timer('b6'):
        if height and width:
            if retPointCloud:
                return (filtered_pc_map, filtered_uvs, filtered_uvs_mask)
            else:
                return filtered_uvs_mask
        else:
            return filtered_pc_map
        

# given bin_num like 0: (means bin_A), return the pointcloud inside
def getObjectPointCloud(req):
    global cachedPCmsg, cachedPCmsgFlag
        
    print "Returning pointclouds"
    bin_num = req.bin_num # bin_num is of the format like 'bin_A'
    #obj_id = req.obj_id #(string)
    
    pc = PointCloud()
    pts = []
    
    with mutex:
        if cachedPCmsgFlag:
            source_pc2_kinect = cachedPCmsg  # does it do deep copy?
        else:
            rospy.logerr("No pointcloud received yet, check your kinect")
    
    # convert PointCloud2 to PointCloud manually
    for point in point_cloud2.read_points(source_pc2_kinect, skip_nans=False):
        pts.append(list(point)[0:3])

    filtered_pc_map = filterPointCloud(bin_num, pts, source_pc2_kinect.header)
    
    return GetObjectPointCloudResponse(filtered_pc_map)

def _min_max(uvs, j):
    _min = 1000000
    _max = -1
    for i in range(len(uvs)):
        if uvs[i][j] > _max:
            _max = uvs[i][j]
        
        if uvs[i][j] < _min:
            _min = uvs[i][j]
    
    return (_min, _max)

def createPointCloud2(pc, uvs):
    # pc: PointCloud
    # uvs: the u,v pairs associated to the pc
    # transform PointCloud into an organized PointCloud2 msg
    
    with Timer('c0'):
        minu, maxu = _min_max(uvs, 0)
        minv, maxv = _min_max(uvs, 1)
        height = maxv - minv + 1
        width = maxu - minu + 1
    
    cnt = 0
    points = []
    length = len(uvs)
    nan = float('nan')
    nans = [nan, nan, nan]
    with Timer('c1'):
        for i in xrange(minu, maxu+1):
            for j in xrange(minv, maxv+1):
                if cnt < length and uvs[cnt][0] == i and uvs[cnt][1] == j:
                    points.append([pc.points[cnt].x, pc.points[cnt].y, pc.points[cnt].z])
                    cnt += 1
                else:
                    points.append(nans)
                
    
    with Timer('c2'):
        # pack the numbers into binary array
        from sensor_msgs.msg import PointCloud2, PointField
        from sensor_msgs.point_cloud2 import _get_struct_fmt
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1)]

        cloud_struct = struct.Struct(_get_struct_fmt(False, fields))

        buff = ctypes.create_string_buffer(cloud_struct.size * len(points))

        point_step, pack_into = cloud_struct.size, cloud_struct.pack_into
        offset = 0
    
        for p in points:
            pack_into(buff, offset, *p)
            offset += point_step


    with Timer('c3'):
        return PointCloud2(header=pc.header,
                       height=height & 0xffffffff,
                       width=width & 0xffffffff,
                       is_dense=False,
                       is_bigendian=False,
                       fields=fields,
                       point_step=cloud_struct.size,
                       row_step=(cloud_struct.size * width) & 0xffffffff,
                       data=buff.raw)

# given bin_num like 0: (means bin_A), return the pointcloud2 inside
def getObjectPointCloud2(req):
    global cachedPCmsg, cachedPCmsgFlag
    global tfListener
        
    print "Returning pointclouds"
    bin_num = req.bin_num # bin_id is of the format like 'bin_A'
    #obj_id = req.obj_id #(string)
    
    pc = PointCloud()
    pts = []
    
    with Timer('a1'):
        with mutex:
            if cachedPCmsgFlag:
                source_pc2_kinect = cachedPCmsg  # does it do deep copy?
            else:
                rospy.logerr("No pointcloud received yet, check your kinect")
                
    # convert PointCloud2 to PointCloud manually
    with Timer('a2'):
        for point in point_cloud2.read_points(source_pc2_kinect, skip_nans=False):
            pts.append(list(point)[0:3])

    with Timer('a3'):
        filtered_uvs_mask = filterPointCloud(bin_num, pts, copy.deepcopy(source_pc2_kinect.header), source_pc2_kinect.height, source_pc2_kinect.width, retPointCloud=False)
        
    # with Timer('a4'):
        # filtered_pc2_map = createPointCloud2(filtered_pc_map, filtered_uvs)
    
    # with Timer('a5'):
        # pub = rospy.Publisher('filtered_pc2_map', sensor_msgs.msg.PointCloud2, queue_size=10)
        # rospy.sleep(1)
        # filtered_pc2_map.height = filtered_pc2_map.height & 0xffffffff
        # filtered_pc2_map.width = filtered_pc2_map.width & 0xffffffff
        # pub.publish(filtered_pc2_map)
    
    # print 'filtered_pc2_map.height', filtered_pc2_map.height
    # print 'filtered_pc2_map.width', filtered_pc2_map.width
    
    return GetObjectPointCloud2Response(source_pc2_kinect, filtered_uvs_mask)


def createPointsMarker(points, marker_id, frame_id, rgba=(1,0,0,1)):
    global vis_pub
    rospy.sleep(0.1)
    
    marker = Marker()
    marker.id = marker_id
    marker.header.frame_id = frame_id
    marker.type = marker.POINTS
    marker.scale.x = 0.01
    marker.scale.y = 0.01
    marker.scale.z = 0.01
    marker.color.a = rgba[3]
    marker.color.r = rgba[0]
    marker.color.g = rgba[1]
    marker.color.b = rgba[2]
    marker.points = points  
    
    vis_pub.publish(marker)

def startPointCloudService():
    s = rospy.Service('pointcloud_service', GetObjectPointCloud, getObjectPointCloud)
    s = rospy.Service('pointcloud2_service', GetObjectPointCloud2, getObjectPointCloud2)

def callback(msg):
    with mutex:
        global cachedPCmsg, cachedPCmsgFlag
        cachedPCmsg = msg
        cachedPCmsgFlag = True
    
def initListener():
    global tfListener
    tfListener = TransformListener()
    input_topic = rospy.get_param('~input_topic')
    rospy.Subscriber(input_topic, sensor_msgs.msg.PointCloud2, callback, queue_size=1)

def getBinMouths(bin_cnstr):
    nrows = len(bin_cnstr)
    ncols = 2
    
    position = [[0 for x in range(ncols)] for x in range(nrows)]
    
    for i in range(nrows):
        x = (bin_cnstr[i][0] + bin_cnstr[i][1])/2
        z = (bin_cnstr[i][4] + bin_cnstr[i][5])/2
        position[i][0] = x
        position[i][1] = z
        
    return(position)


if __name__ == '__main__':
    global cachedPCmsg, cachedPCmsgFlag, vis_pub
    cachedPCmsg = PointCloud2()
    cachedPCmsgFlag = False
    
    rospy.init_node('getpc', anonymous=True)
    initListener()
    
    vis_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    
    # self testing
    selftest = False
    if selftest == True:
        rospy.sleep(1)
        req = GetObjectPointCloudRequest()
        req.bin_num = 5
        with Timer('getObjectPointCloud'):
            ret = getObjectPointCloud(req)
        
        req = GetObjectPointCloud2Request()
        req.bin_num = 5
        with Timer('getObjectPointCloud2'):
            ret = getObjectPointCloud2(req)
    
    createService = True
    if createService:
        startPointCloudService()
        rospy.spin()
    
