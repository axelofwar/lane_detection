## _Camera Calibration: How to _lane_calibrate.cpp_

[![N|Solid](https://cldup.com/dTxpPi9lDf.thumb.png)](https://nodesource.com/products/nsolid)

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

In order to successfully complete lane detection, we will need a to complete a few things first. This file will focus on how to execute the camera calibration to generate calibration statistcs. These statistics will be used to configure the lane detection camera and determine a distance per pixel value that relates the Camera perspective and frame to the world perspective and frame. 

1.) Navigate to the catkin_ws zip file and extract to a location on memory
2.) Navigate to terminal and cd into the catkin_ws directory
3.) catkin_make and confirm build - diagnose build errors as needed
4.) Open catkin_ws/src/laneDetection/include/ and confirm "ChessboardImages" folder is present - replace imageset if needed
5.) Open catkin_ws/src/laneDetection/src/lane_calibrate.cpp
- Change variable "string::path" to reflect the appropriate ChessBoardImages directory in your workspace
- Change board width and height to reflect your Chessboard image intrinsics 
- Change square_size to reflect your Chessboard image intrinsics

6.) Run 'roscore' to initialize ros-master node
```
roscore
```
7.) Run lane_calibrate to generate Calibration Statistics
```
rosrun laneDetection lane_calibrate
```
8.) Wait for completion - terminal will output:
```
"Done Calibration"
```
9.) Navigate to catkin_ws and confirm "Calibration Statistics" yaml file created

