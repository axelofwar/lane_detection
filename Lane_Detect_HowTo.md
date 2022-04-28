# Lane Detection: How to run _lane_find.cpp_

[![N|Solid](https://cldup.com/dTxpPi9lDf.thumb.png)](https://nodesource.com/products/nsolid)

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

In order to run the lane_find.cpp we first need to change ensure a few things:
1.) Make sure that usb camera is plugged in and recognized 
- Navigate to terminal and type:
```
cd /dev 
```
```
ls
```
NOTE: if video0 (zero) is present then usb camera comfirmed

2.) Run catkin_make and update dependencies until sucessful build 
- Install ros_usb_cam package if needed
- Install openCV in if needed 
- Install image_transport dependencies if needed

3.) Upon successful build run in terminal
```
roslaunch laneDetection laneDetection.launch
```
NOTE: DO NOT RUN ROSCORE - roslaunching this launch file will auto instantiate the usb camera and start a roscore node 
#### Variables user can Manipulate
- Change input file size (default set to 640 x 480)
- Increase int min_area to reduce the intensity of pre-processing and noise removal - IF NEEDED
- Decrease "___size" variables according to image and use case - with respect to mask, morph, and connectivity size - IF NEEDED

4.) Monitor popup windows for output integrity! The left image non CV labeled image is using blue-red calculation whereas the right image relies on openCV's grayscale function. The second output is the likely output that we will move forward with