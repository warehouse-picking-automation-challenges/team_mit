CALIBRATION STEPS
=====================

1. Place the robot.
    1. With the help of a measuring tape and a rope we draw a line parallel to the shelf for the robot legs to be flushed
    (300 mm =30 cm on both sides from side face of the shelf to side face of the robot base).
    2. Mark the center of the shelf on that line.
    3. Mark the location of the legs in that line (symmetric from the shelf center) (need distance). Or the center.
    (1040 mm =104 cm = from the front face of the front leg of the shelf to the face of the robot base leg)
    4. Place the robot with the pallet jack.

2. Setup computers.
    1. Sync the time of all computers using NTP tools.
    2. Git pull and catkin_make on all machines
    3. Config computer names correctly in ```robot_perception.launch```, ```kinect2_bridge_2.launch```
       for kinect1, kinect2, and realsense.
    4. Config computer names correctly in ```apc_environment.sh```
       for kinect1, kinect2, and realsense.
    5. Make a link to gamesetting on desktop
    6. Set computer timezone
    7. Turn off visualization (optional)
    8. Update capsen models in mcube-002

3. Bin calibration.
    1. Mount bin calibration tool on robot. 
    2. Run bin calibration script and teleop to touch the bottoms and four sides of bins A,C,D,F,G,I,J,L.
    3. Run script to find the location of the bin and populate the table on bin openings.
    4. Remove bin calibration tool.
    
4. Shelf tracking.
    1. Mount calibration pads on shelf.
    2. Mount cone calibration tool on robot.
    3. Jog robot to locate AprilTags on shelf.
    4. Locate AprilTags with respect to shelf frame.
    5. Remove cone calibration tool.
 
5. Calibrate Kinects
    1. Mount calibration AprilTag holder on robot.
    2. Run script to calibrate both Kinects.
        Teleop to 4 corners of the bin and run two scripts in each.
    3. Remove calibration April Tag holder.
    
6. Calibrate Real Sense.
    1. Run script to move the robot to bin H (empty or with something).
    2. Manual fit (in rviz) of the location of the camera based on pointcould.
    3. Set the parameters in ```percept.py:159```

7. Calibrate pressure sensor.

8. Validate Shelf calibration 
    1. Run primitives for extream conditions.
        1. Scoop
        2. Grasp
        3. Suction
