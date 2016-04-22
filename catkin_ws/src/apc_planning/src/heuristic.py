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



def jsonTest(heuristic, strategies, jsonFilename, useVision = False, withPause = False, isExecute = True, toSort = False, forceSucc = False):
    globalSpeedSet = 'yolo'
    ik.ik.setSpeedByName(globalSpeedSet)
    objectiveBinPos = [1.15, 0 , 0.2]
    
    import json
    from pprint import pprint
    with open(jsonFilename) as data_file:   
        data = json.load(data_file)
    
    if useVision:
        capsen.capsen.init()
    
    bin_contents_all = data['bin_contents']
    work_order = data['work_order']
    if toSort:
        work_order = sortOrder(work_order, bin_contents_all)
    displayOrder(work_order, bin_contents_all)
        
    goToHome.goToHome(robotConfig=None, homePos = [1,0,1.2], isExecute = isExecute, withPause = withPause)
    
    count_suck  = 0
    count_grasp = 0
    for orderInd, order in enumerate(work_order):
        displayOrder(work_order, bin_contents_all, orderInd)
        ik.ik.setSpeedByName(globalSpeedSet)
        count_try = 0
        try_max   = 10
        isDirty = False
        bin_id = order['bin']        # e.g. bin_A
        binNum = bin_id2num(bin_id) # e.g. 0
        obj_id = order['item']       # object's id in string
        
        print '\n%s\n[Heuristic] [Order %d] bin_id:' % ('-'*70, orderInd), bin_id, binNum, '; obj_id:', obj_id , '\n%s' % ('-'*70)
        
        if useVision:
            try:
                obj_pose, pose_type = percept.percept(obj_id=obj_id, binNum=binNum, bin_contents = bin_contents_all[bin_id], isExecute = isExecute, withPause = withPause)

                if obj_pose is None:
                    print '[Heuristic] vision failed, continue to next order.'
                    continue
            except:
                print '[Heuristic] vision failed, continue to next order.', 'encounters errors:', traceback.format_exc()
                continue
        else:
            obj_pose, pose_type = percept.perceptFromManualFit(obj_id)
            
        
        print '[Heuristic] pose_type:', pose_type, '; pose:', map(shortfloat, obj_pose)
        
        
        resetStrat = True
        tryFlag    = True
        visionSucc = True
        
        while resetStrat and tryFlag and visionSucc:
            resetStrat = False
            
            strategy_list = heuristic[obj_id][pose_type]
            for i in range(len(strategy_list)):
                strategy_name = strategy_list[i]
                print '[Heuristic] Attempt', i, 'of', try_max, ':', strategy_name
                
                count_try += 1
                if count_try == try_max:
                    print '[Heuristic] reached maximum trial number'
                    tryFlag = False
                    break
                
                if strategy_name == 'percept' or strategy_name == 'percept-hand':
                    if isDirty:
                        print '[Heuristic] perception has been called'
                        if useVision:
                            ik.ik.setSpeedByName(globalSpeedSet)
                            obj_pose, pose_type_new = percept.percept(obj_pose = obj_pose, binNum = binNum, obj_id = obj_id, bin_contents = bin_contents_all[bin_id], 
                                                            isExecute = isExecute, withPause = withPause)
                        else:
                            obj_pose, pose_type_new = percept.perceptFromManualFit(obj_id)

                        if obj_pose is None:
                            visionSucc = False
                            break
                        else:
                            isDirty = False
                        
                        print '[Heuristic] pose_type:', pose_type_new, '; pose:', obj_pose
                            
                        if pose_type_new == pose_type:
                            pose_type = pose_type_new
                            continue
                        else:
                            pose_type = pose_type_new
                            resetStrat = True
                            break
                else:
                    try:
                        if strategy_name in strategies:
                            goToMouth.goToMouth(robotConfig=None, binNum = binNum, isExecute = True, withPause = withPause)
                            
                            (isDirty,succ) = strategies[ strategy_name ](obj_pose, binNum, obj_id, bin_contents_all[bin_id],
                                                  isExecute = isExecute, withPause = withPause)   #run it
                                                  
                            if forceSucc and (i == len(strategy_list)-1 or count_try == try_max-1): ###### for testing in virtual
                                succ = True
                        else:
                            print '[Heuristic] Strategy', strategy_name , 'not implemented yet. Skip it.'
                            continue
                            
                        if succ: 
                            print '[Heuristic] Strategy', strategy_name , 'success'
                            ik.ik.setSpeedByName('faster')
                            print '[Heuristic] Object HAS been picked up with the %s primitive' % strategy_name
                            if strategy_name == 'suck-down' or strategy_name == 'suck-side':
                                count_suck = count_suck + 1
                                attemptGoToBin = goToBin.goToBin(  robotConfig=None, objectiveBinPos = objectiveBinPos, isExecute = isExecute, 
                                                    withPause = withPause, withSuction = True, counter = count_suck)
                                                    
                            else:
                                count_grasp = count_grasp + 1
                                attemptGoToBin = goToBin.goToBin( binNum=binNum, robotConfig=None, objectiveBinPos = objectiveBinPos, isExecute = isExecute, 
                                                    withPause = withPause, withSuction = False, counter = count_grasp)
                                if not attemptGoToBin:
                                    goToHome.goToHome(robotConfig=None, homePos = [1,0,1.2], isExecute = isExecute, withPause = withPause)
                                    goToBin.goToBin( binNum=binNum, robotConfig=None, objectiveBinPos = objectiveBinPos, isExecute = isExecute, 
                                                withPause = withPause, withSuction = False, counter = count_grasp)
                            break
                    except:
                        print '[Heuristic] Strategy', strategy_name, 'encounters errors:', traceback.format_exc()
                
                    if withPause:  raw_input('[Heuristic] Press any key to continue next strategy.')
                
            ik.ik.setSpeedByName(globalSpeedSet)
            goToHome.goToHome(robotConfig=None, homePos = [1,0,1.2], isExecute = isExecute, withPause = withPause)
            
        if withPause:  raw_input('[Heuristic] Press any key to continue next object.')
    
    print '\n[Heuristic] We are done!'

def main(argv=None):
    if argv is None:
        argv = sys.argv
    
    parser = optparse.OptionParser()
    parser.add_option('--hfilename', action="store", dest='hfilename', help='The heuristic file path', 
                      default=os.environ['APC_BASE']+'/doc/strategy/heuristic-05-2.xlsx')
                      
    parser.add_option('--jfilename', action="store", dest='jfilename', help='The path to json file describing the order', 
                      default=os.environ['APC_BASE']+'/input/apc.json')
                      
    parser.add_option('-v', '--vision', action="store_true", dest='useVision', help='To use vision or not', 
                      default=False)
    
    parser.add_option('-p', '--pause', action="store_true", dest='withPause', help='To pause or not', 
                      default=False)
    
    parser.add_option('-n', '--noexe', action="store_false", dest='isExecute', help='To execute or not', 
                      default=True)
                      
    parser.add_option('-s', '--sort', action="store_true", dest='toSort', help='To sort the orders based on successrate', 
                      default=False)
        
    parser.add_option('-f', '--forcesucc', action="store_true", dest='forceSucc', help='To force success on trying the last strategy in the strategy list; This is useful for testing system in virtual environment.', 
                      default=False)
        
    rospy.init_node('heuristic', anonymous=True)
    
    (opt, args) = parser.parse_args()
    if not os.path.isfile(opt.hfilename):
        opt.hfilename = os.environ['APC_BASE']+'/doc/strategy/'+opt.hfilename
        
    if not os.path.isfile(opt.jfilename):
        opt.jfilename = os.environ['APC_BASE']+'/input/'+opt.jfilename
    
    heuristic = loadHeuristic(filename=opt.hfilename, offset=(6,1), endrow=73, max_strategy_num=10)  
    # offset points to the starting grid in the excel file, e.g. (6,1) = A6, 73 means the endrow+1

    s = getStrategies()
        
    jsonTest(heuristic, s, opt.jfilename, opt.useVision, opt.withPause, opt.isExecute, opt.toSort, opt.forceSucc)
        
        
if __name__=='__main__':
    sys.exit(main())
    
