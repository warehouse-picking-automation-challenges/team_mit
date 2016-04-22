#!/usr/bin/env python

import roslib
roslib.load_manifest('image_capture')
import rospy
from cv_bridge import CvBridge
import sensor_msgs.msg
import cv2
#from threading import Thread, Lock

ind = 0
savepath = ""
def callback(data):
    br = CvBridge()
    im = br.imgmsg_to_cv2(data)
    cv2.namedWindow('Image capture preview', cv2.WINDOW_NORMAL)
    cv2.imshow('Image capture preview', im)
    global ind
    if cv2.waitKey(100) > 0:  # -1 means no key hit
        ind = ind + 1
        print "Capture img%03d.jpg" % ind
        cv2.imwrite("%s/img%03d.jpg" % (savepath, ind),im)
    
    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("Image", sensor_msgs.msg.Image, callback, queue_size=1)
    
    global savepath
    savepath = rospy.get_param("savepath")
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
    
