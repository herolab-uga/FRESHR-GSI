# FRESHR-GSI
We propose a new framework and safety scale for evaluating safety in human-robot interaction settings, where mobile robots share larger regions with human personnel. It is an open-ended solution with modular addition of safety factors as per the application’s need. We develop an online (real-time) quantitative framework to measure the safety level of robots during an interactive task and evaluate the safety of humans. FRESHR is flexible in terms of the applicability of any safety scale and deployment at different endpoints from where safety can be observed. The framework uses a deep learning method for human(s) and robot(s) detection, and algebraic calculations using the detection data to extract the safety-related factors.

Below is the architecture of our framework.

![Alt text](/images/framework_architecture.png?raw=true)


## Pre-requisite
This package is tested on [ROS (noetic)](http://wiki.ros.org/noetic/Installation/Ubuntu). You can use this link to install ROS Noetic into your system [http://wiki.ros.org/noetic/Installation/Ubuntu]. We use [Husky](http://wiki.ros.org/Robots/Husky) in gazebo to do the experiments. To install Husky packages, follow the instruction using this link[http://wiki.ros.org/Robots/Husky]. And finally, we use [Yolov7](https://github.com/WongKinYiu/yolov7) to detect and localize the pixel locations of the human and robots (objects) from RGB images. For a human, it also provides the skeleton’s keypoint locations. These are then correlated with the corresponding depth values from depth images. Yolov7 also provides confidence scores of each detection, which are valuable when integrating different detections of the same factor, such as different skeletal keypoint distances. We have used [lukazso](https://github.com/lukazso/yolov7-ros.git) github repository and made some minor modification to extract the pixel locations with confidence. You can follow the instruction provided in this link [https://github.com/lukazso/yolov7-ros.git] to install yolov7 package in your ROS workspace.


## Setup

For experiments, we use husky in gazebo simulation mounted with intel realsense RGB-D camera. After installing [Husky Packages](http://wiki.ros.org/Robots/Husky), copy the gazebo world files to husky_gazebo/worlds folder and launch files to husky_gazebo/launch folder. These world files are to create human actors as per the cases, scenarios, and settings and launch files will help to launch those world files. Cases and scenarios are defined below:

  Cases :
  * 1 : Using Interactive robot's camera data to provide safety level for Humans.
  * 2 : Using external vision based system to provide safety level for Humans. This case will help humans to move around the environment more freely as the humans do not need to always remain in the viewable range of the interacting robots.
  
  Scenarios :
  * 1 : Only Robot is moving and Human is static.
  * 2 : Only Human is moving and Robot is static.
  * 3 : Both Robot and Human are moving with some velocity.
  
  Settings :
  * 1 : Approaching trajectory is being followed. Human/Robot approaches towards each other and goes back to their origin.
  * 2 : Horizontal trajectory is followed. Human/Robot crosses each other horizontally at a distance. Human might not remain in the viewable range of the interacting robot.
  * 3 : Diagonal trajectory is followed. Human/Robot crosses each other diagonally. Human might not remain in the viewable range of the interacting robot.


As mentioned above for yolov7 we used third party github repo and made some minor changes so, after installing yolov7 copy the yolov7-ros/src and yolov7-ros/launch files from this github to the installed yolov7-ros/src folder and yolov7-ros/launch folder respectively. These python files will publish human skeleton keypoint's pixel location along with confidence in /human_skeleton_keypoints rostopic and bounding box x-y location in /bboxes_array rostopic.

## Installation

After completing all the above steps and making sure everything is installed and working properly. Below are the commands which will help you install FRESHR-GSI at your catkin workspace.
```
  cd ~/catkin_ws
  cd src
  git clone https://github.com/herolab-uga/FRESHR-GSI.git
  cd ../..
  catkin_make
  catkin_make install
```
## ROS Nodes

Yolov7 :
  
  When interacting robot is using this framework, we can directly use the Yolov7 pose estimation to estimate the pose and extract the skeleton keypoints with confidence. But, if we plan to use external vision system to provide GSI value using the framework, we will need the simple object detection scripts to detect both the robot and human. So, we have keypoint.py and object_detect.py to detect the keypoints as well as robot. keypoint.py and object_detect.py files subscribes to the RGB image supllied from the camera and will publish **/human_skeleton_keypoints** and **/bboxes_array** respectively.

Gazebo :
  
  To test this FRESHR-GSI, we use Husky in gazebo simulations. We provided the world files for Human actors movement and launch files to launch the actors and husky in an empty world. Different launch files are present in gazebo/launch folder and each launch file belongs to specific case and scenario. For e.g. case1_sc1.launch is for all settings using case 1 and scenario 1 setup, case1_sc2_1.launch file is for setting 1 for scenario 2 using case 1 setup and so on. Similarly, for spawning the husky we have different launch files which will launch the husky at a scpecific location and especially for case 2 we need two husky to represent one as interacting and one as external agent observing the interaction between human and robot.

Ground_truth : 
  
  To compare FRESHR provided metrics with the ground truth, we have ground_truth.py file which subscribe to the **/gazebo/model_states** and calculate the ground truth distance between human and robot as well as the velocity of each model.

Metric Estimator :
  
  This node is to estimate the distance and velocity of each keypoint of a human skeleton provided by the yolo. This node subscribe to the **/human_skeleton_keypoints** and camera depth data to estimate the metrics for each pixel location present in the /human_skeleton_keypoints rostopic. If the human is not in the viewable range it will simply store NaN values in distance and velocity. This node also publishes **/distance**, **/velocity**, **/confidence** and **/human_detection**. **/confidence** stores the confidence of each keypoints detected and **/human_detection** stores 1 if the human is detected, otherwise -1 will be stored.


GSI :
  
  Generalizable Safety Index is generalizable in terms of safety metrics and the utility of this scale. GSI vlaues from 0 to 1, where 0 means unsafe and 1 means safe. GSI can be used for path planning and safely navigating through a crowded environment. It can be used by an operator or external agent to monitor the safety of the human during any HRI/HRC, and will also allow controlling the robot to provide more safety and comfort to the humans. Our initial focus is on the distance and velocity factors, as they are sufficient measures to determine safety given the large area of interaction space. However, in the future, such as in a human-robot collaboration scenario where the interaction area is smaller, more refined measurements need to be added to the framework for a holistic safety evaluation. These additional measures include relative acceleration of joints and physiological metrics (such as heartbeat or some form of human stress measurements). This node subscribe to **/distance**, **/velocity**, and **/confidence** from metric estimator and publishes below rostopics :
    
    - /freshr/distance_min : This topic provides the minimum distance among all the keypoint's distances estimated by the metric estimator.
    - /freshr/distance_max : This topic provides the maximum distance among all the keypoint's distances estimated by the metric estimator.
    - /freshr/distance_avg : This topic provides the average distance among all the keypoint's distances estimated by the metric estimator.
    - /freshr/distance_wtd : This topic provides the weighted distance among all the keypoint's distances estimated by the metric estimator. Weight is based on the confidence of each keypoint.
    - /freshr/velocity_min : This topic provides the minimum velocity among all the keypoint's velocities estimated by the metric estimator.
    - /freshr/velocity_max : This topic provides the maximum velocity among all the keypoint's velocities estimated by the metric estimator.
    - /freshr/velocity_avg : This topic provides the average velocity among all the keypoint's velocities estimated by the metric estimator.
    - /freshr/velocity_wtd : This topic provides the weighted velocity among all the keypoint's velocities estimated by the metric estimator. Weight is based on the confidence of each keypoint.
    - /freshr/distance_factor : This topic publishes the distance factor using weighted distance.
    - /freshr/velocity_factor : This topic publishes the velocity factor using weighted velocity.
    - /freshr/gsi_dist : This topic publishes the GSI value only based on distance factor, which means that the weight for distance factor is 1 for this topic.
    - /freshr/gsi_vel : This topic publishes the GSI value only based on velocity factor, which means that the weight for velocity factor is 1 for this topic. 
    - /freshr/gsi_dist_wtd : This topic publishes the GSI value based on distance and velocity factor, here the weights for distance factor is 0.75 and 0.25 for velocity factor.
    - /freshr/gsi_vel_wtd : This topic publishes the GSI value based on distance and velocity factor, here the weights for distance factor is 0.25 and 0.75 for velocity factor.
    - /freshr/gsi_avg : This topic publishes the GSI value based on distance and velocity factor, here the weight for distance factor and velocity factor is equal which is 0.5. 
    - /freshr/safety : This topic converts the GSI value to human readable format by using 7-point Likert scale. If there is no human in the camera range, then it will publish "Nothing Detected or System Unavailable".

DI :
  
  DI is the Danger Index. We use this as one of our baseline for comparison. This is based on the paper [https://link.springer.com/article/10.1007/s10514-006-9009-4]. This node also subcribes to the same topic as GSI and provide us the Danger Index. Danger Index is opposite of GSI, which means that 0 means safe and 1 means unsafe, so we conver the scale for comparison by subtracting the DI value from 1. This node publishes the **/danger_index** topic.

ZI :
  
  ZI is zonal based index. This is based on paper [https://doi.org/10.48550/arXiv.2208.02010]. Here they have used an average speed of human and combined it with the distance. They have divided the region into three safety zones : **Red** means unsafe zone (< 0.5 m for comparison purpose), **Yellow** means Neutral zone (<2.25 m and >0.5 m for comparison purpose), and **Green** means safe zone (> 2.25 m for comparison purpose). This node publishes the **/zonal_safety** topic.
  
We have assumed values for few parameters that are fixed. As we will use Magni robot for real-world experiment and are using intel realsense camera we have kept Dmax (maximum distance beyond which everyone is safe) as 5 m. Dmin (minimum distance which should be breached by the robot at any cost) as 0.5 m. Vmax ( maximum velocity of the robot) as 2 m/s. Vmin (minimum velocity of the robot) as -2 m/s. amax (maximum acceleration of the robot) as 0.7 m/s^2.


## Usage Examples

First step to run this project is to start the gazebo simulation. You can run any setting of any case and scenario. For example, if you want to run case 1 and scenario 1 for all setting use the below command.

```
  roslaunch husky_gazebo case1_sc1.launch
```

or if you want to run case 1 scenario 2 for setting 1, use below command.

```
  roslaunch husky_gazebo case1_sc2_1.launch
```
Above command will start the gazebo and you can see a human actor and a husky in an empty world. Now, we will run the yolov7 to estimate the pose of the human actor. Use the below command in a new terminal to estimate the human skeleton keypoints which uses our modified file from yolov7-ros. **Note**, yolov7 needs graphics (cuda) to detect human keypoints so make sure you have cuda installed in your system.

```
  roslaunch yolov7_ros keypoint.launch
```

Yolov7 publishes the Human Skeleton Keypoints at higher rates which will force the FRESHR-GSI to publish same value multiple times. To control the publishing rate you can use [topic-tools](http://wiki.ros.org/topic_tools/throttle) to throttle the pose estimation messages at 1 Hz or use the below command directly (considering you are publishing the keypoints to rostopic /human_skeleton_keypoints) to publish it at 1 Hz.

```
  rosrun topic_tools throttle messages /human_skeleton_keypoints 1.0
```

Before going any further just check if the rostopic **/human_skeleton_keypoints_throttle** is supplying the keypoints location at 1 HZ. After starting the yolov7 open a new terminal and run the below command to estimate the distance and velocity of each keypoints. For velocity, we have just calculated the change in distance over change in time.

```
  roslaunch freshr metric_estimator.launch
```

Then, use the below command to run the all the scales at once.

```
  roslaunch freshr safety_scale.launch
```

This file runs all three scale (GSI, DI, and ZI) at once and you can find the GSI, DI, and ZI related rostopics by doing the **rostopic list** 




