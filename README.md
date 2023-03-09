# phri-safety-index
A new safety index for pHRI applications. It is an open-ended solution with modular addition of safety factors as per the application’s need. We develop an online (real-time) quantitative framework to measure the safety level of robots during an interactive task and evaluate the safety of humans. FRESHR is flexible in terms of the applicability of any safety scale and deployment at different endpoints from where safety can be observed. The framework uses a deep learning method for human(s) and robot(s) detection, and algebraic calculations using the detection data to extract the safety-related factors.

Below is the architecture of our framework.

![Alt text](/FRESHR/architecture.png?raw=true)


## Pre-requisite
This package is tested on ROS-noetic and it is assumed that ROS (noetic) is already installed in your system. This package uses Yolov7 for the human(s) pose estimation. You can download the yolov7 package from its official website. 
