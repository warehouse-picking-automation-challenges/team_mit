#!/usr/bin/python

import json
import sys
from apc.helper import loadHeuristic
import optparse
import os

def genTest1(heuristic, ntrial = 1, outfilename = 'out.json'):
    
    data = {}
    bin_id = "bin_I"
    data["bin_contents"] = {bin_id: [""]}  # a dummy
    
    testObjList=["oreo_mega_stuf",
                "crayola_64_ct",
                "paper_mate_12_count_mirado_black_warrior",
                "mead_index_cards",
                "laugh_out_loud_joke_book",
                "expo_dry_erase_board_eraser",
                "feline_greenies_dental_treats",
                "dr_browns_bottle_brush",
                "highland_6539_self_stick_notes",
                "genuine_joe_plastic_stir_sticks",
                "cheezit_big_original",
                "elmers_washable_no_run_school_glue"]
    
    work_order = []
        
    for obj_id in testObjList:
        for pose_type, strategy_list in heuristic[obj_id].iteritems():
            for i in range(ntrial):
                order = {'bin': bin_id, 'item': obj_id, 'pose_type': pose_type, 'trial_num': i+1}
                work_order.append(order)

    data["work_order"] = work_order

    with open(outfilename, 'w') as outfile:
        json.dump(data, outfile, sort_keys = True, indent = 4, ensure_ascii=False)
    

def main(argv=None):
    if argv is None:
        argv = sys.argv
        
    parser = optparse.OptionParser()
    parser.add_option('--hfilename', action="store", dest='hfilename', help='The heuristic file path', 
                      default=os.environ['APC_BASE']+'/doc/strategy/heuristic-04-23.xlsx')
                      
    parser.add_option('-n', action="store", dest='ntrial', help='N times', type="int",
                      default='12')
                      
    parser.add_option('-o', action="store", dest='outfilename', help='Out file name', 
                      default='out.json')
                      
    (opt, args) = parser.parse_args()
                      
    heuristic = loadHeuristic(filename=opt.hfilename, offset=(6,1), endrow=73)  
    
    genTest1(heuristic, opt.ntrial, opt.outfilename)
        

if __name__=='__main__':
    sys.exit(main())
