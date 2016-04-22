#!/usr/bin/python

# bench marking realsense with captured data

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
import tf 
import json

def get_immediate_subdirectories(a_dir):
    return [name for name in os.listdir(a_dir)
            if os.path.isdir(os.path.join(a_dir, name))]
            
def get_immediate_files(a_dir):
    return [name for name in os.listdir(a_dir)
            if os.path.isfile(os.path.join(a_dir, name))]

def main():
    
    rospy.init_node('capsen_benchmark', anonymous=True)
    
    basefolder = os.getenv('APCDATA_BASE') + '/bagfiles/vision_dataset/Bin0/'
    outputfile = os.getenv('APCDATA_BASE') + '/bagfiles/vision_dataset/Bin0/result.json'
    pc_topic = '/realsense/depth_registered/points'
    bin_num = 0
    mode = 1  # 0 for kinect, 1 for realsense
    item_names = get_immediate_subdirectories(basefolder)
    pubTf = rospy.Publisher('tf', TFMessage, queue_size=10)
    pubPointCloud = rospy.Publisher(pc_topic, PointCloud2, queue_size=1)
    capsen.init()
    
    listener = tf.TransformListener()
    rospy.sleep(0.1)
    
    result = []
    for item_name in item_names:
        #os.system("echo '%s' > %s/capsen_vision/models/models.txt" % (item_name, os.getenv('APCDATA_BASE')) ) # hack
        
        object_folder = os.path.join(basefolder, item_name)
        pose_types = get_immediate_subdirectories(object_folder)
        for pose_type in pose_types:
            posefolder = os.path.join(object_folder, pose_type)
            bagFilenames = get_immediate_files(posefolder)
            
            for bagFilename in bagFilenames:
                bagpath = os.path.join(posefolder, bagFilename)
                bag = rosbag.Bag(bagpath)
                print bagpath
                
                cnt = 0
                for topic, msg, t in bag.read_messages(topics=[pc_topic]):
                    for i in range(3):
                        msg.header.stamp = rospy.Time.now()
                        pubPointCloud.publish(msg)
                        rospy.sleep(0.1)
                    rospy.sleep(0.5)

                    pose = detectOneObject(item_name, [item_name], bin_num, mode)
                    
                    if pose is not None:
                        print item_name, find_object_pose_type(item_name, pose), pose_type, bagFilename, cnt
                        result.append([item_name, pose, find_object_pose_type(item_name, pose), pose_type, bagFilename, cnt])
                    else:
                        print item_name, 'None', pose_type, bagFilename, cnt
                        result.append([item_name, None, None, pose_type, bagFilename, cnt])
                    cnt += 1
                    #raw_input('press enter to continue')
                with open(outputfile, 'w') as outfile:
                    json.dump(result, outfile)
        
    os.system("cp %s/capsen_vision/models/models_full.txt %s/capsen_vision/models/models.txt" % (os.getenv('APCDATA_BASE'), os.getenv('APCDATA_BASE'))) # hack
        
if __name__ == "__main__":
    sys.exit(main())
