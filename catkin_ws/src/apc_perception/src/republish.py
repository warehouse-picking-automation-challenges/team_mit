#!/usr/bin/env python

import roslib
import rospy
import sensor_msgs.msg
import sys

counter = 0
pub = None
def callback(data):
    global counter
    global pub
    data.header.seq = counter
    counter = counter +1
    data.header.stamp = rospy.Time.now()
    
    pub.publish(data)
    
def listener():
    global pub
    rospy.init_node('listener', anonymous=True)
    
    argv = rospy.myargv(argv=sys.argv)
    if len(argv) != 4:
        print 'Usage: republish.py [topic_name_src] [topic_name_out] [type]'
        
    topic_name_src = argv[1]
    topic_name_out = argv[2]
    type_name = eval(argv[3])
    
    pub = rospy.Publisher(topic_name_out, type_name, queue_size=1)
    rospy.Subscriber(topic_name_src, type_name, callback, queue_size=1)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
    
