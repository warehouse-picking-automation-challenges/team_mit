
import rospy
import chan    # python channel https://chan.readthedocs.org/en/latest/
import geometry_msgs.msg
import tf.transformations as tfm
import tf
import traceback

ntfretry = 40
retryTime = 0.05
secBefore = 0.5

class ROS_Wait_For_Msg:
    #init_node=True if you call use this class outside of an rosnode
    def __init__(self, topic_name, msgtype, init_node=False):
        self.init_node = init_node
        self.data = None
        self.channel = chan.Chan()
        self.topic_name = topic_name
        self.msgtype = msgtype
        
        if self.init_node:
            rospy.init_node('listener', anonymous=True)

    def __del__(self):
        if self.init_node:
            rospy.signal_shutdown('ROS_Wait_For_Msg done')

    def callback(self, data):
        self.channel.put(data)
        self.sub.unregister()   

    def getmsg(self):
        self.sub = rospy.Subscriber(self.topic_name, self.msgtype, self.callback)
        #print 'Waiting in ROS_Wait_For_Msg for %s...' % self.topic_name
        return self.channel.get()
        # do we need to remove the rosnode
        

def ros2matlabQuat(qros):  # qxqyqzqw -> qwqxqyqz
    qmat = [qros[3]]
    qmat.extend(qros[0:3])
    return qmat

from geometry_msgs.msg import Pose
def poselist2pose(poselist):
    pose = Pose()
    pose.position.x = poselist[0]
    pose.position.y = poselist[1]
    pose.position.z = poselist[2]
    pose.orientation.x = poselist[3]
    pose.orientation.y = poselist[4]
    pose.orientation.z = poselist[5]
    pose.orientation.w = poselist[6]
    return pose

def pose2list(pose):
    return [pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

def pubFrame(br, pose=[0,0,0,0,0,0,1], frame_id='obj', parent_frame_id='world', npub=5):
    if len(pose) == 7:
        ori = tuple(pose[3:7])
    elif len(pose) == 6:
        ori = tfm.quaternion_from_euler(*pose[3:6])
    else:
        print 'Bad length of pose'
        return 
    
    pos = tuple(pose[0:3])
    
    for j in range(npub):
        rospy.sleep(0.01)
        br.sendTransform(pos, ori, rospy.Time.now(), frame_id, parent_frame_id)

#there is a ROS library that does things for you. You can query this library
#and it will do math for you.
#pt is a point
#homeFrame is a string labeling the frame we care about
#targetFrame is a string see above
#homeFrame is a frame XYZ list (list of list, it's kind of like a matrix)
#targetFrame is also a frame XYZ list
#listener is a ROS listener
def coordinateFrameTransform(pt, homeFrame, targetFrame, listener):
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = homeFrame
    #while not rospy.is_shutdown():
    for i in range(ntfretry):
        try:
            if ntfretry % 2 == 0:
                t = listener.getLatestCommonTime(targetFrame, homeFrame)
            else:
                t = rospy.Time.now() - rospy.Time.from_sec(secBefore)
            pose.header.stamp = t
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = pt[2]
            pose_target = listener.transformPose(targetFrame, pose)
            return pose_target
        except: #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print '[coordinateFrameTransform] failed to transform'
            print '[coordinateFrameTransform] targetFrame %s homeFrame %s' % (targetFrame, homeFrame)
            print traceback.format_exc()
            print '[coordinateFrameTransform] retry', i
            rospy.sleep(retryTime)

# given a pose in list, transform to target frame and return a list
# coordinateFrameTransform return a PoseStamped
def poseTransform(pose, homeFrame, targetFrame, listener):
    _pose = geometry_msgs.msg.PoseStamped()
    _pose.header.frame_id = homeFrame
    if len(pose) == 6:
        pose.append(0)
        pose[3:7] = tfm.quaternion_from_euler(pose[3], pose[4], pose[5]).tolist()
    
    _pose.pose.position.x = pose[0]
    _pose.pose.position.y = pose[1]
    _pose.pose.position.z = pose[2]
    _pose.pose.orientation.x = pose[3]
    _pose.pose.orientation.y = pose[4]
    _pose.pose.orientation.z = pose[5]
    _pose.pose.orientation.w = pose[6]
    #while not rospy.is_shutdown():
    for i in range(ntfretry):
        try:
            if ntfretry % 2 == 0:
                t = listener.getLatestCommonTime(targetFrame, homeFrame)
            else:
                t = rospy.Time.now() - rospy.Time.from_sec(secBefore)
            _pose.header.stamp = t
            _pose_target = listener.transformPose(targetFrame, _pose)
            p = _pose_target.pose.position
            o = _pose_target.pose.orientation
            return [p.x, p.y, p.z, o.x, o.y, o.z, o.w]
        except: #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print '[poseTransform] failed to transform'
            print '[poseTransform] targetFrame %s homeFrame %s' % (targetFrame, homeFrame)
            print traceback.format_exc()
            print '[poseTransform] retry', i
            rospy.sleep(retryTime)

# given a ROS Pose(), transform to target frame and return as a ROS Pose()
def rosposeTransform(pose, homeFrame, targetFrame, listener):
    _pose = geometry_msgs.msg.PoseStamped()
    _pose.header.frame_id = homeFrame
    #while not rospy.is_shutdown():
    for i in range(ntfretry):
        try:
            if ntfretry % 2 == 0:
                t = listener.getLatestCommonTime(targetFrame, homeFrame)
            else:
                t = rospy.Time.now() - rospy.Time.from_sec(secBefore)
            t = listener.getLatestCommonTime(targetFrame, homeFrame)
            _pose.header.stamp = t
            _pose.pose = pose
            _pose_target = listener.transformPose(targetFrame, _pose)
            return _pose_target.pose
        except: #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print '[rosposeTransform] failed to transform'
            print '[rosposeTransform] targetFrame %s homeFrame %s' % (targetFrame, homeFrame)
            print traceback.format_exc()
            print '[rosposeTransform] retry', i
            rospy.sleep(retryTime)

def lookupTransform(homeFrame, targetFrame, listener):
    #while not rospy.is_shutdown():
    for i in range(ntfretry):
        try:
            if ntfretry % 2 == 0:
                t = listener.getLatestCommonTime(targetFrame, homeFrame)
            else:
                t = rospy.Time.now() - rospy.Time.from_sec(secBefore)
            (trans,rot) = listener.lookupTransform(homeFrame, targetFrame, t)
            return (trans,rot)
        except: #(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print '[lookupTransform] failed to transform'
            print '[lookupTransform] targetFrame %s homeFrame %s' % (targetFrame, homeFrame)
            print traceback.format_exc()
            print '[lookupTransform] retry' , i
            rospy.sleep(retryTime)
    
    
