#!/usr/bin/python

import os
import rosbag
import pdb
import sys
import rospy
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2
from capsen import detectOneObject
import capsen
from ik.helper import find_object_pose_type
import traceback

def get_immediate_subdirectories(a_dir):
    return [name for name in os.listdir(a_dir)
            if os.path.isdir(os.path.join(a_dir, name))]
            
def get_immediate_files(a_dir):
    return [name for name in os.listdir(a_dir)
            if os.path.isfile(os.path.join(a_dir, name))]

def main():
    
    rospy.init_node('capsen_benchmark', anonymous=True)
    
    basefolder = os.getenv('APCDATA_BASE') + '/bagfiles/vision_dataset/Bin0/'
    pc_topic = '/realsense/depth_registered/points'
    bin_num = 0
    mode = 1  # 0 for kinect, 1 for realsense
    item_names = get_immediate_subdirectories(basefolder)
    pubTf = rospy.Publisher('tf', TFMessage, queue_size=10)
    pubPointCloud = rospy.Publisher(pc_topic, PointCloud2, queue_size=10)
    capsen.init()
    
    
    for item_name in item_names:
        object_folder = os.path.join(basefolder, item_name)
        pose_types = get_immediate_subdirectories(object_folder)
        for pose_type in pose_types:
            posefolder = os.path.join(object_folder, pose_type)
            bagFilenames = get_immediate_files(posefolder)
            
            while True:
            #for bagFilename in bagFilenames:
                bagFilename = bagFilenames[0]
                bagpath = os.path.join(posefolder, bagFilename)
                bag = rosbag.Bag(bagpath)
                #print bagpath
                for topic, msg, t in bag.read_messages(topics=['/tf']):
                    if topic == '/tf':
                        rospy.sleep(0.003)
                        for i in range(len(msg.transforms)):
                            msg.transforms[i].header.stamp = rospy.Time.now()
                        pubTf.publish(msg)
        
if __name__ == "__main__":
    sys.exit(main())
