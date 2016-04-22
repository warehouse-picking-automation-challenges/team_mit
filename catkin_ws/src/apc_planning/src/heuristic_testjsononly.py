#!/usr/bin/python

import os
import goToBin
import goToHome
import goToMouth
import scoop
import push
import topple
import grasp
import sys
import geometry_msgs.msg
import std_msgs
import json
import tf
import ik.ik
from ik.roshelper import ROS_Wait_For_Msg
from ik.ik import IK
from ik.helper import find_object_pose_type
from ik.helper import shortfloat
from apc.helper import loadHeuristic
from apc.helper import sortOrder
from apc.helper import displayOrder
import rospy
import pdb
import numpy as np
import math
import tf.transformations as tfm
import gripper
import capsen.capsen
import optparse
import plan_suckdown_func
import suction_down
import suction_side
import percept
import goToHomeSuction
import push_rotate
import suction_side
import flush_grasp
import traceback

def getStrategies():
    # todo extend strategies here
    strategies = {'scoop' : scoop.scoop,
                  'push' : push.push,
                  'topple' : topple.topple,
                  'grasp' : grasp.grasp,
                  'suck-down' : suction_down.suction_down,
                  'percept' : percept.percept,
                  'push-rotate' : push_rotate.push_rotate,
                  'suck-side' : suction_side.suction_side,
                  'percept-hand' : percept.percept}
    return strategies

def bin_id2num(bin_id):
    return ord(bin_id[4:])-ord('A')  # bin_id is of the format 'bin_A'
    
def jsonTest(heuristic, strategies, jsonFilename, useVision = False, withPause = False, isExecute = True, toSort = False):
    objectiveBinPos = [1.15, 0 , 0.2]
    
    import json
    from pprint import pprint
    with open(jsonFilename) as data_file:   
        data = json.load(data_file)
    
    
    bin_contents_all = data['bin_contents']
    work_order = data['work_order']
    if toSort:
        work_order = sortOrder(work_order, bin_contents_all)
    displayOrder(work_order, bin_contents_all)
        
    for orderInd, order in enumerate(work_order):
        #displayOrder(work_order, bin_contents_all, orderInd)
        
        bin_id = order['bin']        # e.g. bin_A
        binNum = bin_id2num(bin_id) # e.g. 0
        obj_id = order['item']       # object's id in string
        
        print '\n%s\n[Heuristic] [Order %d] bin_id:' % ('-'*70, orderInd), bin_id, binNum, '; obj_id:', obj_id , '\n%s' % ('-'*70)
        
            
        strategy_list = heuristic[obj_id]
                
                            
    
    print '\n[Heuristic] We are done!'

def main(argv=None):
    if argv is None:
        argv = sys.argv
    
    parser = optparse.OptionParser()
    parser.add_option('--hfilename', action="store", dest='hfilename', help='The heuristic file path', 
                      default=os.environ['APC_BASE']+'/doc/strategy/heuristic-05-2.xlsx')
                      
    parser.add_option('--jfilename', action="store", dest='jfilename', help='The path to json file describing the order', 
                      default=os.environ['APC_BASE']+'/game_settings/JSON/apc.json')
                      
    parser.add_option('-v', '--vision', action="store_true", dest='useVision', help='To use vision or not', 
                      default=False)
    
    parser.add_option('-p', '--pause', action="store_true", dest='withPause', help='To pause or not', 
                      default=False)
    
    parser.add_option('-n', '--noexe', action="store_false", dest='isExecute', help='To execute or not', 
                      default=True)
                      
    parser.add_option('-s', '--sort', action="store_true", dest='toSort', help='To sort the orders based on successrate', 
                      default=False)
        
    rospy.init_node('heuristic', anonymous=True)
    
    (opt, args) = parser.parse_args()
    if not os.path.isfile(opt.hfilename):
        opt.hfilename = os.environ['APC_BASE']+'/doc/strategy/'+opt.hfilename
        
    if not os.path.isfile(opt.jfilename):
        opt.jfilename = os.environ['APC_BASE']+'/game_settings/JSON/'+opt.jfilename
    
    heuristic = loadHeuristic(filename=opt.hfilename, offset=(6,1), endrow=73, max_strategy_num=10)  
    # offset points to the starting grid, e.g. (6,1) = A6, 73 means the endrow+1

    s = getStrategies()
        
    #interactiveTest(heuristic, s)
    jsonTest(heuristic, s, opt.jfilename, opt.useVision, opt.withPause, opt.isExecute, opt.toSort)
        
        
if __name__=='__main__':
    sys.exit(main())
    
