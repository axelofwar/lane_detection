# lane_detection_tyler
catkin_ws with code and markdown files for lane detection scripts. lane_detect = static &amp; lane_find = dynamic

lane_detection_ws.zip is the zip file including all output files as well as calibration files. Included individually are the pertinent files for lane detection.

The path planning and localization algorithms that I contributed to are proprietary to the UDM IGVC group, and thus are not included. 

The outdated_IP_code is written in tandem with Philip Renn and Dr. Mark Paulik. Other files are completely written by me and part of a vehicle upgrade (not all of which are mine to share).

**FOR THOSE FAMILIAR WITH ROS**
```
catkin_create_pkg 
```
to make a new ROS package with catkin dependencies. Move the relevant packages into the src, launch, and include folders.
```
rosdep install
```
this will install all missing and necessary dependencies if files were properly added to src, include, and launch folders

```
roscore
```
this will start the ros network (assuming it's installed)
```
catkin_make
```
this wil build the package. make sure a USB camera is plugged in before running launch file OR run indiviudally. 
```
roslaunch lane_detection.launch
```
OR
```
rosrun lane_detect.cpp
```
```
rosrun lane_find.cpp
``````
```
rosrun etc.cpp
```
Use these rosrun finctions in order to run the files within. Once I clean up the directory from all the integration files that were proprietary to the other parts of the project a cleaner push to git can be made. 
