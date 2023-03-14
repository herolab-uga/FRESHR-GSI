# phri-safety-index
A new safety index for pHRI applications. It is an open-ended solution with modular addition of safety factors as per the application’s need. We develop an online (real-time) quantitative framework to measure the safety level of robots during an interactive task and evaluate the safety of humans. FRESHR is flexible in terms of the applicability of any safety scale and deployment at different endpoints from where safety can be observed. The framework uses a deep learning method for human(s) and robot(s) detection, and algebraic calculations using the detection data to extract the safety-related factors.

Below is the architecture of our framework.

![Alt text](/images/architecture.png?raw=true)


## Pre-requisite
Husky packages [http://wiki.ros.org/Robots/Husky]
Yolov7 [https://github.com/lukazso/yolov7-ros.git]

This package is tested on [ROS (noetic)](http://wiki.ros.org/noetic/Installation/Ubuntu). We use [Yolov7](https://github.com/WongKinYiu/yolov7) to detect and localize the pixel locations of the human and robots (objects) from RGB images. For a human, it also provides the skeleton’s keypoint locations. These are then correlated with the corresponding depth values from depth images. Yolov7 also provides confidence scores of each detection, which are valuable when integrating different detections of the same factor, such as different skeletal keypoint distances.


## Setup
gazebo -- launch and world files
yolov7 - changes to third party code

## Installation
To use these packages you will have to install this package into your ROS workspace. Make sure you install ROS Noetic and set up your ROS workspace as per the instructions at [http://wiki.ros.org/noetic/Installation/Ubuntu].  Below are the commands which will help you do that, considering you already have a catkin workspace.
```
  cd ~/catkin_ws
  cd src
  git clone https://github.com/herolab-uga/FRESHR-GSI.git
  cd ../..
  catkin_make
  catkin_make install
```
## ROS Nodes

Yolov7 keypoint and bounding box extraction

ground truth from gazebo

Metric Estimator - distance and velocity

GSI scale

DI scale

ZI scale
## Test Examples

Yolov7 publishes the Human Skeleton Keypoints at higher rates which will force the FRESHR-GSI to publish same value multiple times. To control the publishing rate you can use [topic-tools](http://wiki.ros.org/topic_tools/throttle) to throttle the pose estimation messages at 1 Hz or use the below command directly (considering you are publishing the keypoints to rostopic /human_skeleton_keypoints) to publish it at 1 Hz.
```
  rosrun topic_tools throttle messages /human_skeleton_keypoints 1.0
```

Above command will publish your rostopic /your_rostopic_name to a new rostopic named as /your_rostopic_name_throttle. 

This package has three ROS nodes
  1) dist_vel.py : This node is just to collect the ground truth data from the gazebo simulations
  2) distance.py : This node is subscribing to the human skeleton keypoints and the camera depth data. Using these data it estimates the distance and velocity of each keypoints along with the confidence value. If the human is not in the viewable range, it will store NaN values on distance and velocity. Finally it will publish the distance, velocity and confidence of each keypoints in a separate array. Along with this it will also publish 1 in /human_detected topic if the human is detected at that instance otherwise it will print -1.
  3) compatibility_scale.py : This node subscribes to the distance, velocity, confidence topics and uses the below equation to calculate the GSI values.

After starting the yolov7 open a new terminal and run the below command to estimate the distance and velocity of each keypoints

```
  roslaunch cv_basics distance.py
```
You need to make sure that you are subscribing to the right topic names in this file. Run the "rostopic list" to make sure /distance /velocity and /confidence are being published. If not, then there might be some issue with the subscribing part.

Then, use the below command to run the compatibility scale node.
```
  roslaunch cv_basics compatibility_scale.py
```
This node subscribes rostopics /distance /velocity and /confidence to supply you the below mentioned rostopics:

  1) /Framework/distance_min :
  2) /Framework/distance_max :
  3) /Framework/distance_avg :
  4) /Framework/distance_wtd :
  5) /Framework/velocity_min :
  6) /Framework/velocity_max :
  7) /Framework/velocity_avg :
  8) /Framework/velocity_wtd :
  9) /Framework/distance_factor :
  10) /Framework/velocity_factor :
  11) /Framework/safety_value_dist :
  12) /Framework/safety_value_vel : 
  13) /Framework/safety_value_dist_wtd :
  14) /Framework/safety_value_vel_wtd :
  15) /Framework/safety_value_avg : 
  16) /Framework/safety :

