#!/usr/bin/env python

import rospy
import tf
import geometry_msgs
from std_msgs.msg import String
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates 
import numpy as np
import math
import time

global actor_prev_pose_x,actor_prev_pose_y,actor_prev_pose_z,actor_velocity,prev_time, rel_velocity, prev_distance, robot_velocity,robot_prev_pose_x,robot_prev_pose_y,camera_offset
actor_prev_pose_x = -10.0
actor_prev_pose_y = -10.0
robot_prev_pose_x = -10.0
robot_prev_pose_y = -10.0
prev_time = -1.0
actor_velocity = 100.0
current_distance = -1.0
rel_velocity = 100.0
robot_velocity = 100.0
prev_distance = 100.0
camera_offset = 0.3
time_d = 0.0


def transform_callback(data):
    
    global actor_prev_pose_x,actor_prev_pose_y,actor_prev_pose_z,actor_velocity,prev_time, current_distance, rel_velocity, prev_distance, robot_velocity,robot_prev_pose_x,robot_prev_pose_y,camera_offset
    time_now = rospy.get_rostime().to_sec()
    
    quaternion = (data.pose[2].orientation.x,data.pose[2].orientation.y,data.pose[2].orientation.z,data.pose[2].orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    husky_position_y = data.pose[2].position.y
    husky_position_x = data.pose[2].position.x
    
    if str(yaw).startswith('-1.57'):
    	husky_position_y = data.pose[2].position.y - camera_offset
    	
    elif str(yaw).startswith('1.57'):
    	husky_position_y = data.pose[2].position.y + camera_offset
    	
    elif str(yaw).startswith('0'):
    	husky_position_x = data.pose[2].position.x + camera_offset
    	
    elif str(yaw).startswith('3.14'):
    	husky_position_x = data.pose[2].position.x - camera_offset
    	
    	
    actor_position_x = data.pose[1].position.x
    actor_position_y = data.pose[1].position.y

    current_distance = math.sqrt(math.pow(actor_position_x - husky_position_x, 2) + math.pow(actor_position_y - husky_position_y,2))
    time_diff = (time_now - prev_time)
    distance = 0.0

    if actor_prev_pose_x != -10.0 and actor_prev_pose_y != -10.0 and prev_time != -1.0 :
        distance = math.sqrt(math.pow(actor_position_x - actor_prev_pose_x, 2) + math.pow(actor_position_y - actor_prev_pose_y,2))
        actor_velocity = distance/time_diff
    
    if robot_prev_pose_x != -10.0 and robot_prev_pose_y != -10.0 and prev_time != -1.0 :
        distance1 = math.sqrt(math.pow(husky_position_x - robot_prev_pose_x, 2) + math.pow(husky_position_y - robot_prev_pose_y,2))
        robot_velocity = distance1/time_diff
    
    if current_distance != -1.0:
    	rel_velocity = (prev_distance - current_distance)/time_diff
    
    actor_prev_pose_x = actor_position_x
    actor_prev_pose_y = actor_position_y
    robot_prev_pose_x = husky_position_x
    robot_prev_pose_y = husky_position_y
    prev_distance = current_distance
    prev_time = time_now
    time_d = time_diff
    print(current_distance, actor_velocity, robot_velocity, rel_velocity)


def dxl_control():
    
    global actor_prev_pose_x,actor_prev_pose_y,actor_prev_pose_z,actor_velocity,prev_time, current_distance, rel_velocity, prev_distance, robot_velocity,robot_prev_pose_x,robot_prev_pose_y
    rospy.init_node('distance_velocity', anonymous=True)
    act_pub = rospy.Publisher('/GT_actor_velocity', Float64, queue_size = 10)
    rob_pub = rospy.Publisher('/GT_robot_velocity', Float64, queue_size = 10)
    rel_vel_pub = rospy.Publisher('/GT_rel_velocity', Float64, queue_size = 10)
    dist_pub = rospy.Publisher('/GT_Distance', Float64, queue_size = 10)
    rate = rospy.Rate(1)
    rospy.Subscriber('/gazebo/model_states_throttle', ModelStates, transform_callback)
    while not rospy.is_shutdown():     
        dist_pub.publish(Float64(current_distance))
        act_pub.publish(Float64(actor_velocity))
        rob_pub.publish(Float64(robot_velocity))
        rel_vel_pub.publish(Float64(rel_velocity))
        rate.sleep()


if __name__ == '__main__':
    try:
        dxl_control()
    except rospy.ROSInterruptException:
        pass
