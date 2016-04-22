import os
import json
import sys
import pdb
import numpy as np
import tf
import rospy
import tf.transformations as tfm
import time
from numpy import linalg as la
import traceback
from roshelper import lookupTransform


listener = None

def get_obj_dim(objId):
    jsonFilename = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/object_data/objectDictionary.json'
    with open(jsonFilename) as data_file:
        data = json.load(data_file)
    
    object_dim = data[objId]
    object_dim[0]/=1000.0 #object_dim[0]=object_dim[0]/1000.0
    object_dim[1]/=1000.0
    object_dim[2]/=1000.0
    return object_dim
    
def get_obj_capsentf(objId):
    jsonFilename = os.environ['APC_BASE']+'/catkin_ws/src/apc_config/object_data/objectCapsenTransform.json'
    with open(jsonFilename) as data_file:
        data = json.load(data_file)
    
    return data[objId]

def xyzrpy_from_xyzquat(pose):
    return pose[0:3] + list(tfm.euler_from_quaternion(pose[3:7])) # x,y,z,qx,qy,qz,qw

def matrix_from_xyzquat(translate, quaternion):
    return np.dot(tfm.compose_matrix(translate=translate) , 
                   tfm.quaternion_matrix(quaternion)).tolist()

def transformBack(tf_xyzquat, pose):
    T_mat = tfm.concatenate_matrices( tfm.translation_matrix(tf_xyzquat[0:3]), tfm.quaternion_matrix(tf_xyzquat[3:7]))
    pose_mat = tfm.concatenate_matrices( tfm.translation_matrix(pose[0:3]),  tfm.quaternion_matrix(pose[3:7]) )
    new_pose_mat = np.dot(pose_mat, tfm.inverse_matrix(T_mat))
    return tfm.translation_from_matrix(new_pose_mat).tolist() + tfm.quaternion_from_matrix(new_pose_mat).tolist()

# need to move into roshelper
def find_object_pose_type(objId= 'crayola_64_ct',objPose=[1.95,0.25,1.4,0,0,0,1]):
    #objId:string: Id of the object
    #objPose : list : is a list of 7 numbers showing x y z and quaternion (x y z w)
    global listener
    
    if objId=='kyjen_squeakin_eggs_plush_puppies' or objId=='kong_duck_dog_toy' or objId=='kong_sitting_frog_dog_toy':
        obj_pose_type='other'
    else:
        obj_dim=get_obj_dim(objId)
        obj_dim=np.array(obj_dim)
        
        #~ *****************************************************************
        # initialize listener rospy
        if listener is None:
            listener = tf.TransformListener()
            rospy.sleep(0.1)
        
        (shelf_position, shelf_quaternion) = lookupTransform('/shelf', '/map', listener)
        
                
        #print t
        #~ print  shelf_position, shelf_quaternion 
        
        #~ *****************************************************************

        # Convert xyx quat to tranformation matrix for Shelf frame

        shelf_pose_tfm_list=matrix_from_xyzquat(shelf_position,shelf_quaternion)
        shelf_pose_tfm=np.array(shelf_pose_tfm_list)

        shelf_pose_orient=shelf_pose_tfm[0:3,0:3]
            
        #Zaxis of the shelf frame
        shelf_Z=shelf_pose_orient[:,2]
        
        #~ *****************************************************************
        
        # Convert xyx quat to tranformation matrix for Object frame
        
        obj_pose_tfm_list=matrix_from_xyzquat(objPose[0:3], objPose[3:7])
        obj_pose_tfm=np.array(obj_pose_tfm_list)
        
        obj_pose_orient=obj_pose_tfm[0:3,0:3]
        
        #~ *****************************************************************
        
        #Find projection of object axes on Z axis of the shelf frame    
        #To find out which object frame is lying closer to the Z axis of the shelf
            
        proj_vecZ=np.dot(shelf_Z,obj_pose_orient)
            
        max_proj_valZ,Zaxis_dir=np.max(np.fabs(proj_vecZ)), np.argmax(np.fabs(proj_vecZ))
        
        #~ *****************************************************************
        m = {0: 'upright', 1: 'intermediate', 2:'flat'}
        obj_pose_type=m[Zaxis_dir]

    if objId=='rolodex_jumbo_pencil_cup':
        obj_pose_type = 'upright'
        # if obj_pose_type == 'upright':
            #print 'It is hard to differentiate between upright and upsidedown , so setting pose type to "upright".'
            #Find angle of the edge of the object 
            # Cos_angle_made_with_shelf_Z=proj_vecZ[Zaxis_dir]/(la.norm(shelf_Z)*la.norm(obj_pose_orient[:,Zaxis_dir]))
                                # 
            # angle_to_shelfZ=np.arccos(Cos_angle_made_with_shelf_Z)*180/np.pi
            # 
            # if angle_to_shelfZ<90:
                # obj_pose_type='upright'
            # else:
                # obj_pose_type='upsidedown'
        # 
        # else:
            # shelf_Y=shelf_pose_orient[:,1]        
            # proj_vecY=np.dot(shelf_Y,obj_pose_orient)
            # max_proj_valY,hand_norm_dir=np.max(np.fabs(proj_vecY)), np.argmax(np.fabs(proj_vecY))
        # 
            # if hand_norm_dir ==0:
                # #print 'It is hard to differentiate between mouth-front and mouth-back, so setting pose type to "mouth-front".'
                # #~ obj_pose_type='mouth-front'
                        # 
                # #Find angle of the edge of the object 
                # Cos_angle_made_with_shelf_Y=proj_vecY[hand_norm_dir]/(la.norm(shelf_Y)*la.norm(obj_pose_orient[:,hand_norm_dir]))
                                # 
                # angle_to_shelfY=np.arccos(Cos_angle_made_with_shelf_Y)*180/np.pi
            # 
                # if angle_to_shelfY<90:
                    # obj_pose_type='mouth-front'
                # else:
                    # obj_pose_type='mouth-back'
            # 
            # else:
                # obj_pose_type='mouth-side'
        
    if objId=='kong_air_dog_squeakair_tennis_ball' and obj_pose_type != 'flat':
        obj_pose_type='other'
        
    if objId=='dr_browns_bottle_brush' and obj_pose_type != 'flat':
        obj_pose_type='other'
        
    if objId=='munchkin_white_hot_duck_bath_toy' and obj_pose_type != 'flat':
        obj_pose_type='upright'
        
    if objId=='feline_greenies_dental_treats' and obj_pose_type != 'flat':
        obj_pose_type='upright'
        
    if objId=='elmers_washable_no_run_school_glue' and obj_pose_type != 'flat':
        obj_pose_type='upright'
        
    #print 'The current object obj_pose_type is:', obj_pose_type
    return(obj_pose_type)


# something useful for building primitives
import geometry_msgs.msg

def get_bin_cnstr():
    # right \ # left \ # back \  # front  \ # bottom \ # top
    return   [[0.1554, 0.42926, 0, 0.42, 1.52393, 1.79063],
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

def get_bin_cnstr_world():
    # assume the shelf is in perfect orientation
    # right \ # left \ # back \  # front  \ # bottom \ # top
    bin_cnstr_shelf = get_bin_cnstr()
    

def getBinMouthAndFloor(distFromShelf, binNum):
    #this function gives you XYZ in shelf frame from center of front face of a given bin
    #plus distance from shelf in the depth direction
    #positive values get of distFromShelf get you further out
    bin_cnstr = get_bin_cnstr()
             
    nrows = len(bin_cnstr)
    ncols = 3
    
    position = [0,0,0]
    x = (bin_cnstr[binNum][0] + bin_cnstr[binNum][1])/2
    y = distFromShelf + bin_cnstr[binNum][3]
    z = (bin_cnstr[binNum][4] + bin_cnstr[binNum][5])/2
    position[0] = x
    position[1] = y
    position[2] = z
    binBaseHeight = bin_cnstr[binNum][4]                       
    return(position, binBaseHeight)
    
def getBinMouth(distFromShelf, binNum):
    bin_cnstr = get_bin_cnstr()
             
    nrows = len(bin_cnstr)
    ncols = 3
    
    position = [0,0,0]
    x = (bin_cnstr[binNum][0] + bin_cnstr[binNum][1])/2
    y = distFromShelf + bin_cnstr[binNum][3]
    z = (bin_cnstr[binNum][4] + bin_cnstr[binNum][5])/2
    position[0] = x
    position[1] = y
    position[2] = z
    bin_height = bin_cnstr[binNum][5] - bin_cnstr[binNum][4]
    bin_width  = bin_cnstr[binNum][1] - bin_cnstr[binNum][0]          
    return(position,bin_height,bin_width)
    
def get_bin_inner_cnstr():
    # right \ # left \ # back \  # front  \ # bottom \ # top
    bin_cnstr = get_bin_cnstr()
    SL = 0.02541225  # Side lip Width 
    BL = 0.0295825   # Bottom lip Width 
    TL = 0.0124175   # Top lip width  # estimated, may need robot touch
    bin_cnstr_offset =  [[0, -SL, 0, 0, +BL, -TL],
                         [0, 0, 0, 0, +BL, -TL],
                         [+SL, 0, 0, 0, +BL, -TL],
                         [0, -SL, 0, 0, +BL, -TL],
                         [0, 0, 0, 0, +BL, -TL],
                         [+SL, 0, 0, 0, +BL, -TL],
                         [0, -SL, 0, 0, +BL, -TL],
                         [0, 0, 0, 0, +BL, -TL],
                         [+SL, 0, 0, 0, +BL, -TL],
                         [0, -SL, 0, 0, +BL, -TL],
                         [0, 0, 0, 0, +BL, -TL],
                         [+SL, 0, 0, 0, +BL, -TL]]
    
    new_bin_cnstr = (np.array(bin_cnstr) + np.array(bin_cnstr_offset)).tolist()
    return new_bin_cnstr

def find_shelf_walls(binNum):
    bin_cnstr = get_bin_cnstr()
             
    nrows = len(bin_cnstr)
    ncols = 3
    
    binRightWall = bin_cnstr[binNum][0]
    binLeftWall = bin_cnstr[binNum][1]
    return(binRightWall,binLeftWall)

def getMaxHeight(binNum):
    bin_cnstr = get_bin_cnstr()
             
    nrows = len(bin_cnstr)
    ncols = 3
    maxHeight = bin_cnstr[binNum][5]                       
    return( maxHeight)
    
def getMinHeight(binNum):
    bin_cnstr = get_bin_cnstr()
             
    nrows = len(bin_cnstr)
    ncols = 3
    minHeight = bin_cnstr[binNum][4]                       
    return( minHeight)

def pause():
    raw_input('Press any key to continue')

def pauseFunc(withPause):
    if withPause:
        pause()
    return

def visualizeFunc(withVisualize, plan):
    if withVisualize:
        plan.visualize()
    return
    
def getObjCOM(objPose, objId):
    #gives you the center of mass of the object    
    # object frame is attached at com
    objPosition = objPose[0:3]
    return objPosition

import gripper
def openGripper():
    # call gripper node, open
    gripper.open()
    return 1

def closeGripper(forceThreshold):
    # call gripper node, close
    gripper.close()
    return 1

def graspGripper(move_pos, move_speed=50):
    # move_pos in meter, move_speed in mm/s
    #WE should make grasp gripper force controlled--Nikhil
    # call gripper node, grasp
    move_pos=1000*move_pos
    gripper.move(move_pos, move_speed)
    return 1

def moveGripper(move_pos, move_speed=50):
    # move_pos in meter, move_speed in mm/s
    # call gripper node, grasp
    move_pos=1000*move_pos
    gripper.move(move_pos, move_speed)
    return 1

def setForceGripper(force=50):
    gripper.set_force(force)
    return 1
    
def matrix_from_xyzquat(translate, quaternion):
    return np.dot(tfm.compose_matrix(translate=translate) , 
                   tfm.quaternion_matrix(quaternion)).tolist()


def quat_from_matrix(rot_matrix):
    return (tfm.quaternion_from_matrix(rot_matrix))


def rotmatY(theta):
    theta_rad=(theta*np.pi)/180
    return(np.array([[np.cos(theta_rad), 0, np.sin(theta_rad)],
    [0, 1, 0],[-np.sin(theta_rad), 0, np.cos(theta_rad)]]))


def rotmatX(theta):
    theta_rad=(theta*np.pi)/180
    return(np.array([[1, 0, 0],
    [0, np.cos(theta_rad), -np.sin(theta_rad)],[0, np.sin(theta_rad), np.cos(theta_rad)]]))

def rotmatZ(theta):
    theta_rad=(theta*np.pi)/180
    return(np.array([[np.cos(theta_rad), -np.sin(theta_rad), 0],[np.sin(theta_rad), np.cos(theta_rad), 0], [0,0,1]]))
    
class Timer(object):
    #### Timer to time a piece of code
    def __init__(self, name=None):
        self.name = name

    def __enter__(self):
        self.tstart = time.time()

    def __exit__(self, type, value, traceback):
        if self.name:
            print '\t[%s]' % self.name,
        print 'Elapsed: %s' % (time.time() - self.tstart)

class shortfloat(float):
    def __repr__(self):
        return "%0.3f" % self
