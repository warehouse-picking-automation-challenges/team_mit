#!/usr/bin/python

import json
import os
import sys
import collections

def show(x):
    if x is None:
        return 'None'
    return x

def main():
    
    inputfilepath = os.getenv('APCDATA_BASE') + '/bagfiles/vision_dataset/Bin0_Kinect/result.json'

    with open(inputfilepath, 'r') as infile:
        data = json.load(infile)
        
    # entry: [item_name, pose, detected posetype, true posetype, bagfilename, imagenum]
    total = 0
    succ = 0
    for entry in data:
        if entry[2] == entry[3]:
            succ += 1
        total += 1
    print "Total posetype correctness: ", succ/float(total), '(%d/%d)' % (succ, total)
    
    succObjectPosetype = {}
    totalObjectPosetype = {}
    
    ###########################################
    # succrate for each object
    ###########################################
    
    for entry in data:
        objectposename = entry[0] + '_[' + entry[3] +']'
        if objectposename not in totalObjectPosetype:
            totalObjectPosetype[objectposename] = 0.0
            succObjectPosetype[objectposename] = 0.0
            
        if entry[2] == entry[3]:
            succObjectPosetype[objectposename] += 1
        
        totalObjectPosetype[objectposename] += 1
    
    print "\nEach object posetype correctness:"
    od = collections.OrderedDict(sorted(succObjectPosetype.items()))
    for key in od:
        print key, ',', succObjectPosetype[key]/totalObjectPosetype[key]
    
    #########################################
    
    confuseObjectPosetype = {}
    for entry in data:
        objectposeposename = entry[0] + '_[' + entry[3] + ']_[' + show(entry[2]) + ']'
            
        if entry[2] == entry[3]:
            pass
        else:
            if objectposeposename not in confuseObjectPosetype:
                confuseObjectPosetype[objectposeposename] = 0
            confuseObjectPosetype[objectposeposename] += 1
        
    print "\nEach object posetype confused count:"
    od = collections.OrderedDict(sorted(confuseObjectPosetype.items()))
    for key in od:
        print key, ',', confuseObjectPosetype[key], '/', totalObjectPosetype["_".join(key.split('_')[0:-1])]
    #########################################
        
if __name__ == "__main__":
    sys.exit(main())
