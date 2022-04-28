## _Lane_Track: How to _lane_track.cpp_

[![N|Solid](https://cldup.com/dTxpPi9lDf.thumb.png)](https://nodesource.com/products/nsolid)

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)

In order to successfully complete lane detection, we will need to call the lane tracking node. This node will take input from the perspective transformed image and output the vector<point> objects that will populate the PointCloud2 objects. This node is currently designed to read local image input for testing and training purposes. 

1.) Navigate to the catkin_ws zip file and extract to a location on memory
2.) Navigate to terminal and cd into the catkin_ws directory
3.) catkin_make and confirm build - diagnose build errors as needed
4.) Configure frame input
- a.) Open catkin_ws/src/laneDetection/include/ and include a set of perspective transformed images for training
- b.) Configure ROS_cam callback to subscribe to appropriate image topic

5.) Open catkin_ws/src/laneDetection/src/lane_track.cpp
- Change variable "string::path" to reflect the appropriate transformed lane set directory in your workspace
- Change appropriate sizing paramters should you want somethin other than 640 x 480
- Change value subtracted from "binaryOnes" in order to ajust lowest point image is scanned for initial points if wanted

6.) Run 'roscore' to initialize ros-master node
```
roscore
```
7.) Run lane_calibrate to generate Calibration Statistics
```
rosrun laneDetection lane_track
```
8.) Wait for completion - terminal will output:
- Frame time statistics
- Frame dimension statistics
- Confirmation of detected white points in stats variable non-empty

9.) Observe correct tracked lanes (store locally with imwrite() if necessary)
10.) Name output topic accordingly and publish

