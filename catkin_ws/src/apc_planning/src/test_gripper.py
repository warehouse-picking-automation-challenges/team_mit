#!/usr/bin/env python
import gripper
from ik.helper import Timer

gripper.homing()

gripper.open(speed=50)
with Timer('close50'):
    gripper.close(speed=50)
    
with Timer('open50'):
    gripper.open(speed=50)
    
with Timer('close100'):
    gripper.close(speed=100)
with Timer('open100'):
    gripper.open(speed=100)
    
with Timer('close200'):
    gripper.close(speed=200)
with Timer('open200'):
    gripper.open(speed=200)
    
with Timer('close400'):
    gripper.close(speed=400)
with Timer('open400'):
    gripper.open(speed=400)
#gripper.grasp()
#gripper.grasp(move_pos=10, move_speed=50)
#gripper.release(speed=50)

#gripper.close(50)
