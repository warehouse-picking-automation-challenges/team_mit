import sys
import os
sys.path.append(os.environ['APC_BASE']+"/catkin_ws/src/apc_planning/src/")

import rospy
rospy.init_node('pythonapc', anonymous=True)
import suction
import gripper


