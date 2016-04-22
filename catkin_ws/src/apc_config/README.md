Launch file for apc

Launch all things rviz, robot, kinects
========
```
roslaunch apc_config robot.launch
```

Calibrate kinect
========
1. Start the camera
```
roslaunch apc_config kinect2_bridge.launch
```
2. Calibrate it
```
mkdir ~/kinect_calib_data/tmp; cd ~/kinect_calib_data/tmp
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 record color
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate color
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 record ir
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate ir
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 record sync
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate sync
rosrun kinect2_calibration kinect2_calibration chess5x7x0.03 calibrate depth
```
3. Restart the camera
```
roslaunch apc_config kinect2_bridge.launch
```
4. View the result
```
rosrun registration_viewer view kinect2 sd cloud
```

Visualize your robot in rviz
=======
```
roslaunch urdf_tutorial display.launch model:='$(find apc_config)/models/IRB120/irb_120.urdf'
```