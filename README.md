# FRESHR-GSI
We propose a new framework and safety scale for evaluating safety in human-robot interaction settings, where mobile robots share larger regions with human personnel. It is an open-ended solution with a modular addition of safety factors as per the application’s need. We develop an online (real-time) quantitative framework to measure the safety level of robots during an interactive task and evaluate the safety of humans. FRESHR provides a formal basis to assess and consolidate the safety of multiple humans around a mobile robot. We further provide guidelines in configuring the non-linearity trend with which a factor influences the safety aspects depending on the physical capabilities of mobile robots being evaluated for human safety. It is also flexible in terms of the applicability of any safety scale and deployment at different endpoints from where safety can be observed. The framework uses a deep learning method for human(s) and robot(s) detection, and algebraic calculations using the detection data to extract the safety-related factors.

Below is the architecture of our framework.

![Alt text](/images/framework_architecture.png?raw=true)


## Pre-requisite
This package is tested on [ROS (noetic)](http://wiki.ros.org/noetic/Installation/Ubuntu). You can use this link to install ROS Noetic into your system [http://wiki.ros.org/noetic/Installation/Ubuntu]. We use [Husky](http://wiki.ros.org/Robots/Husky) in the gazebo to do the experiments. To install Husky packages, follow the instructions using this link[http://wiki.ros.org/Robots/Husky]. Finally, we use [Yolov7](https://github.com/WongKinYiu/yolov7) to detect and localize the pixel locations of the humans and robots (objects) from RGB images. For a human, it also provides the skeleton’s keypoint locations. These are then correlated with the corresponding depth values from depth images. Yolov7 also provides confidence scores for each detection, which are valuable when integrating different detections of the same factor, such as different skeletal keypoint distances. We have used [lukazso](https://github.com/lukazso/yolov7-ros.git) GitHub repository and made some minor modifications to extract the pixel locations with confidence. You can follow the instructions provided in this link [https://github.com/lukazso/yolov7-ros.git] to install the yolov7 package in your ROS workspace.


## Setup

For experiments, we use husky in gazebo simulation mounted with intel realsense RGB-D camera. After installing [Husky Packages](http://wiki.ros.org/Robots/Husky), copy the gazebo world files to the husky_gazebo/worlds folder and launch files to the husky_gazebo/launch folder. These world files are to create human actors as per the cases, scenarios, and settings and launch files will help to launch those world files. Cases and scenarios are defined below:

  Cases :
  * 1: Using Interactive robot's camera data to provide a safety level for Humans.
  * 2: Using an external vision-based system to provide a safety level for Humans. This case will help humans to move around the environment more freely as humans do not need to always remain in the viewable range of the interacting robots.
  
  Scenarios :
  * 1: Only the Robot is moving and the Human is static.
  * 2: Only the Human is moving and the Robot is static.
  * 3: Both robots and Humans are moving with some velocity.
  
  Settings :
  * 1: Approaching trajectory is being followed. Human/Robot approaches towards each other and goes back to their origin.
  * 2: Horizontal trajectory is followed. Human/Robot crosses each other horizontally at a distance. Humans might not remain in the viewable range of the interacting robot.
  * 3: Diagonal trajectory is followed. Human/Robot crosses each other diagonally. Humans might not remain in the viewable range of the interacting robot.


As mentioned above for yolov7 we used third-party GitHub repo and made some minor changes so, after installing yolov7 copy the yolov7-ros/src and yolov7-ros/launch files from this github to the installed yolov7-ros/src folder and yolov7-ros/launch folder respectively. These Python files will publish the human skeleton keypoint's pixel location along with confidence in /human_skeleton_keypoints rostopic and bounding box x-y location in /bboxes_array rostopic.

## Installation

After completing all the above steps and making sure everything is installed and working properly. Below are the commands that will help you install FRESHR-GSI at your catkin workspace.
```
  cd ~/catkin_ws
  cd src
  git clone https://github.com/herolab-uga/FRESHR-GSI.git
  cd ../..
  catkin_make
  catkin_make install
```
## ROS Nodes

**Yolov7:**

Our FRESHR-GSI framework enhances interactions between humans and robots by leveraging advanced pose estimation and object detection technologies. This section guides you through utilizing our framework, whether you're directly interacting with robots or integrating external vision systems for enhanced situational awareness.
Direct Robot Interaction

For direct interactions with robots, we utilize YOLOv7 pose estimation. This powerful tool enables us to accurately estimate human poses, extracting skeleton keypoints along with their confidence levels. This direct method is straightforward and highly effective for scenarios where the robot is the primary observer or participant in the interaction.
Integration with External Vision Systems

If your setup involves an external vision system to evaluate the Generalizable Safety Index (GSI) values, additional steps are required to ensure seamless integration:

    Simple Object Detection: To accommodate external vision systems, our framework includes scripts for basic object detection tasks. These are essential for recognizing both human participants and robots within the environment.

    Key Detection Scripts:
        keypoint.py: This script is dedicated to detecting and extracting human skeleton keypoints. It operates on RGB images from the camera, identifying key points of interest on the human body and calculating their confidence levels.
        object_detect.py: Complementing keypoint.py, this script focuses on detecting robots within the camera's field of view. It identifies and outlines robots, facilitating a comprehensive understanding of the scene for safety calculations.

Both keypoint.py and object_detect.py subscribe to RGB image feeds provided by the camera. Upon processing, they publish the detected data to /human_skeleton_keypoints and /bboxes_array topics, respectively. This publication mechanism ensures that all relevant information is readily available for further processing and analysis within the framework.

**Gazebo:**
  
  Our FRESHR-GSI framework has been thoroughly tested using Husky robots within the Gazebo simulation environment. To facilitate this, we've prepared and included world files that animate human actors' movements, alongside launch files that initiate both the actors and a Husky robot within a simulated empty environment. These resources aim to provide a comprehensive setup for testing under various scenarios and conditions.
Launch Files Organization

Located within the gazebo/launch directory, you'll find a series of launch files, each tailored to a specific test case and scenario:

    Case and Scenario Specific Files: Files named following the pattern case#_sc#.launch are configured for distinct setups. For instance, case1_sc1.launch is prepared for testing using the settings of case 1 and scenario 1. Similarly, case1_sc2_1.launch targets the first setting of scenario 2, using the setup from case 1, and so forth.
    Husky Spawning Files: Separate launch files are available for spawning the Husky robot at predetermined locations within the simulation. This setup is crucial for conducting precise and controlled experiments.
    Special Case Configurations: For scenarios requiring the representation of both an interacting and an observing external agent, such as in case 2, we provide configurations for deploying two Husky robots. This setup allows for a detailed examination of interactions between humans and robots, as well as the observation of these interactions by an external entity.

Adding More Human Actors

While the provided world file currently includes a single human actor by default, the framework is designed to accommodate multiple human actors within the simulation environment. This flexibility allows for testing the FRESHR-GSI framework under conditions involving various levels of human-robot interaction and crowd complexity.
Steps for Testing

    Choose Your Scenario: Begin by selecting the appropriate launch file for the scenario you wish to test. The launch file will set up the environment, including the Husky robot(s) and human actors.

    Launch the Simulation: Execute the chosen launch file using ROS's roslaunch command. This step will initiate the Gazebo simulation, placing the Husky robot(s) and human actor(s) according to the scenario's specifications.

    Observe and Analyze: With the simulation running, you can now observe the interactions and collect data relevant to the FRESHR-GSI's performance. This includes assessing the system's ability to accurately calculate safety indexes in dynamic environments.

    Modify and Expand: Feel free to add more human actors to the simulation or tweak the scenario settings to explore the framework's capabilities across a broader range of conditions.

By following these guidelines, researchers and developers can leverage our Gazebo simulation setup to rigorously test and refine the FRESHR-GSI framework, ensuring its effectiveness and reliability in real-world applications.

**Ground_truth:**
  
  To compare FRESHR-provided metrics with the ground truth, we have ground_truth.py file which subscribes to the **/gazebo/model_states** and calculates the ground truth distance between human and robot as well as the velocity of each model.

**Metric Estimator:**
  
  This node is designed to calculate the distance and velocity of individual keypoints in a human skeleton as detected by YOLO. It subscribes to the /human_skeleton_keypoints and camera depth data to derive these metrics for each keypoint coordinate detailed in the /human_skeleton_keypoints ROS topic. In cases where a human is not within the detectable range, the node defaults to recording NaN values for both distance and velocity.

Additionally, this node outputs several topics:

    `/distance`: Estimated distances of keypoints.
    `/velocity`: Velocity of each keypoint.
    `/confidence`: Confidence levels for each detected keypoint.
    `/human_detection`: This topic reflects the total count of humans detected; if no humans are detected, it publishes `-1`.

It also leverages the camera_info topic to utilize the camera's intrinsic values for calculating and publishing the bearing of detected humans within the camera's field of view. These bearings are published as angles in radians to /azimuth_angle_of_humans and in degrees to /degree_angle_of_humans.

This setup ensures a comprehensive assessment of human presence and movement, valuable for applications requiring precise human tracking and analysis.


**GSI:**
  
  The Generalizable Safety Index (GSI) is a versatile metric for assessing safety across various parameters, offering a scalable utility from 0 (unsafe) to 1 (safe). It plays a crucial role in path planning and ensuring secure navigation through populated areas. GSI facilitates both manual and autonomous monitoring of human safety in human-robot interactions (HRI) and human-robot collaboration (HRC), enhancing the comfort and security provided by robots.

Initial evaluations focus on distance and velocity as primary safety indicators within large interaction spaces. Future enhancements will introduce finer measurements for compact spaces, such as joint acceleration and physiological indicators (e.g., heart rate, stress levels), to provide a comprehensive safety analysis.

This node subscribes to metrics from the estimator (/distance, /velocity, /confidence, /azimuth_angle_of_humans) and publishes detailed safety assessments across multiple topics:

    Distance Metrics:
        /freshr/distance_min: Minimum distance from the metric estimator.
        /freshr/distance_max: Maximum distance from the metric estimator.
        /freshr/distance_avg: Average distance from the metric estimator.
        /freshr/distance_wtd: Weighted distance, with weights based on keypoint confidence.

    Velocity Metrics:
        /freshr/velocity_min: Minimum velocity from the metric estimator.
        /freshr/velocity_max: Maximum velocity from the metric estimator.
        /freshr/velocity_avg: Average velocity from the metric estimator.
        /freshr/velocity_wtd: Weighted velocity, with weights derived from keypoint confidence.

    Safety and Comfort Factors:
        /freshr/distance_factor: Publishes the distance factor using weighted distance.
        /freshr/velocity_factor: Publishes the velocity factor using weighted velocity.

    GSI Metrics:
        Various GSI topics publish values based on distance and velocity, employing different weights to represent safety indices accurately. These include specific evaluations like /freshr/gsi_dist for distance, /freshr/gsi_vel for velocity, and combined metrics with varied weighting schemes.

    Readable Safety Evaluation:
        /freshr/safety: Converts GSI values into a human-readable format using a 7-point Likert scale. Reports "Nothing Detected or System Unavailable" if no humans are detected.

    Directional Safety Index:
        /freshr/overall_gsi: Integrates GSI averages with azimuth angles to calculate a directional safety index, prioritizing the safety implications of the robot's movements toward humans.

This framework underscores the importance of adaptive and nuanced safety assessments in robotics, paving the way for safer human-robot ecosystems.

**DI:**
  
  DI is the Danger Index. We use this as one of our baseline for comparison. This is based on the paper [https://link.springer.com/article/10.1007/s10514-006-9009-4]. This node also subscribes to the same topic as GSI and provides us with the Danger Index. The Danger Index is the opposite of GSI, which means that 0 means safe and 1 means unsafe, so we converge the scale for comparison by subtracting the DI value from 1. This node publishes the **/danger_index** topic.

**ZI:**
  
  ZI is the zonal-based index. This is based on the paper [https://doi.org/10.48550/arXiv.2208.02010]. Here they have used the average speed of a human and combined it with the distance. They have divided the region into three safety zones: **Red** means unsafe zone (< 0.5 m for comparison purposes), **Yellow** means Neutral zone (<2.25 m and >0.5 m for comparison purposes), and **Green** means safe zone (> 2.25 m for comparison purpose). This node publishes the **/zonal_safety** topic.
  
We have assumed values for a few parameters that are fixed. As we will use the Magni robot for real-world experiments and are using an intel realsense camera we have kept Dmax (maximum distance beyond which everyone is safe) as 5 m. Dmin (the minimum distance that should be breached by the robot at any cost) is 0.5 m. Vmax ( maximum velocity of the robot) as 2 m/s. Vmin (minimum velocity of the robot) as -2 m/s. amax (maximum acceleration of the robot) as 0.7 m/s^2.


## Usage Examples

The first step to running this project is to start the gazebo simulation. You can run any setting of any case and scenario. For example, if you want to run case 1 and scenario 1 for all settings use the below command.

```
  roslaunch husky_gazebo case1_sc1.launch
```

or if you want to run case 1 scenario 2 for setting 1, use the below command.

```
  roslaunch husky_gazebo case1_sc2_1.launch
```
The above command will start the gazebo and you can see a human actor and a husky in an empty world. Now, we will run the yolov7 to estimate the pose of the human actor. Use the below command in a new terminal to estimate the human skeleton keypoints which uses our modified file from yolov7-ros. **Note**, yolov7 needs graphics (cuda) to detect human keypoints so make sure you have cuda installed in your system.

```
  roslaunch yolov7_ros keypoint.launch
```

Yolov7 publishes the Human Skeleton Keypoints at higher rates which will force the FRESHR-GSI to publish the same value multiple times. To control the publishing rate you can use [topic-tools](http://wiki.ros.org/topic_tools/throttle) to throttle the pose estimation messages at 1 Hz or use the below command directly (considering you are publishing the keypoints to rostopic /human_skeleton_keypoints) to publish it at 1 Hz.

```
  rosrun topic_tools throttle messages /human_skeleton_keypoints 1.0
```

Before going any further just check if the rostopic **/human_skeleton_keypoints_throttle** is supplying the keypoints location at 1 HZ. After starting the yolov7 open a new terminal and run the below command to estimate the distance and velocity of each keypoints. For velocity, we have just calculated the change in distance over the change in time.

```
  roslaunch freshr metric_estimator.launch
```

Then, use the below command to run all the scales at once.

```
  roslaunch freshr safety_scale.launch
```

This file runs all three scales (GSI, DI, and ZI) at once and you can find the GSI, DI, and ZI-related rostopics by doing the **rostopic list** 


## Real World Experiment
 
 To transition our system from a simulated or test environment to real-world experimentation, a key step involves aligning the input data streams with those generated by your physical camera setup. This process ensures that our YOLO model, metric estimator, and Generalizable Safety Index (GSI) node can access and process live data accurately. Specifically, you'll need to update the topics for color images, depth images, and camera information to match those provided by your camera. Here's how to do it:

    Identify Your Camera Topics: Each camera and ROS setup can have unique topic names for publishing color images, depth images, and camera information. For example, standard topics might be named /camera/color/image_raw for color images, /camera/depth/image_rect_raw for depth images, and /camera/color/camera_info for camera information. Your setup might use different names.

    Update YOLO Node Configuration: The YOLO object detection node requires access to color images from your camera to detect and classify objects. Locate the configuration file or parameter settings where the YOLO node subscribes to the color image topic and update it to your camera's color image topic.

    Update Metric Estimator Configuration: The metric estimator calculates distances, velocities, and other metrics based on depth images and camera information. Similar to the YOLO node, find where the metric estimator subscribes to depth image and camera info topics and update these to match your camera's topics.

    Testing and Validation: After updating the topic subscriptions, conduct thorough testing to ensure that the nodes are receiving and processing data correctly. Pay special attention to the data types and formats expected by each node to avoid compatibility issues.

    Troubleshooting: If the nodes do not seem to be functioning as expected after the updates, check the following:
        Are the topic names exactly as published by your camera? Even small typos can cause issues.
        Is the camera properly configured and publishing data on the expected topics?
        Are there any ROS networking issues preventing communication between nodes?

By following these steps and ensuring that each component of our system is correctly configured to receive the appropriate real-world data from your camera, you're setting up a robust foundation for conducting real-world experiments with our YOLO, metric estimator, and GSI framework. This configuration is crucial for accurate detection, metric estimation, and safety indexing in live environments.



