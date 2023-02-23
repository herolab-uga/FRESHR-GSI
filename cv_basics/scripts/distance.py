#!/usr/bin/python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import torch
import cv2
import numpy as np
import time
import rospy
import math
import pyrealsense2 as rs
from std_msgs.msg import String, Float64,Int32MultiArray, Float64MultiArray
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2


#"keypoints": [ "1 nose", "2 right_shoulder", "3 right_elbow", "4 right_wrist", "5 left_shoulder", "6 left_elbow", "7 left_wrist", "8 right_hip", "9 right_knee", "10 right_ankle", "11 left_hip", "12left_knee", "13 left_ankle", "14 right_eye", "15 left_eye", "16 right_ear", "17 left_ear"]

#"keypoints": [ "12 left_wrist", "13 right_wrist", "14 left_hip", "15 right_hip", "16 left_knee", "17 right_knee"]

global distance, velocity, conf, prev_distance, change_in_distance, prev_time, distance1, velocity1, conf1, prev_distance1, prev_time1,pc,offset
offset = 17
pc = 0
distance = [float('nan') for i in range(17)]
distance1 = [float('nan') for i in range(17)]
velocity = [float('nan') for i in range(17)]
velocity1 = [float('nan') for i in range(17)]
change_in_distance = [ float('nan') for i in range(17)]
conf = [float('nan') for i in range(17)]
conf1 = [float('nan') for i in range(17)]
prev_distance = [ float('nan') for i in range(17)]
prev_distance1 = [ float('nan') for i in range(17)]
depth_image = 0
min_dist = -1.0
conf_thr = 0.85
prev_time = -20.0
prev_time1 = -20.0
human_detect = 0
dist_btw = Float64MultiArray()
dist1 = Float64MultiArray()
my_dist = Float64MultiArray()
my_vel = Float64MultiArray()
my_conf = Float64MultiArray()


def get_distance(img):
	global depth_image,depth_img
	bridge=CvBridge()
	depth_image = bridge.imgmsg_to_cv2(img, desired_encoding="32FC1")

def pc_distance(data):
	global pc
	pc = data
	

def robot_keypoint(data):
	global distance1, velocity1, conf1, depth_image, depth_img, prev_distance1, change_in_distance, prev_time1,human_detect,pc, newarr1,offset
	
	robot_data = data.data
	newarr1 = (np.array_split(robot_data, 17))
	time_now1 = rospy.get_rostime().to_sec()
	try:
		for i in range(17):
			#gen = pc2.read_points(self.pc, field_names='z', skip_nans=False, uvs=[(int(newarr[i][0]), int(newarr[i][1]))])
			if math.isnan(depth_image[int(newarr1[i][1]),int(newarr1[i][0])]) == False:
				#gen = pc2.read_points(pc, field_names='z', skip_nans=False, uvs=[(int(newarr[i][0]), int(newarr[i][1]))])
				#print(gen)
				distance1[i] = (depth_image[int(newarr1[i][1]),int(newarr1[i][0])])
				#print(distance[i])
				human_detect = 1
				conf1[i] = newarr1[i][2]
		
		if prev_time1 != -20.0 :
			for i in range(17):
				if prev_distance1[i] == float('nan') or distance1[i] == float('nan'):
					velocity1[i] = float('nan')
					conf1[i] = float('nan')
				else:
					velocity1[i] = (prev_distance1[i] - distance1[i])/(time_now1 - prev_time1)	
	except Exception as e:
		distance1 = [float('nan') for i in range(17)]
		velocity1 = [float('nan') for i in range(17)]
		conf1 = [float('nan') for i in range(17)]
		human_detect = -1
	
	prev_time1 = time_now1
	prev_distance1 = distance1[:]

	#my_dist.data = distance1
	#my_vel.data = velocity1
	#my_conf.data = conf1
	#print("++++++++++++++++++++++++++++++++++++++++++++++++++++++")
	#print("robot dist",distance1)
	#print("robot vel",velocity1)
	#print("robot conf",conf1)
	#print("++++++++++++++++++++++++++++++++++++++++++++++++++++++")
	#pub1.publish(my_dist)
	#pub2.publish(my_vel)
	#pub3.publish(my_conf)
	#pub4.publish(human_detect)

    
def keypoint(data):
	global distance, velocity, conf, depth_image, depth_img, prev_distance, change_in_distance, prev_time,human_detect,pc,newarr1, newarr,offset 
	
	publishing_data = data.data
	newarr = (np.array_split(publishing_data, 17))
	time_now = rospy.get_rostime().to_sec()
	
	#for i in range(17):
	#	
	#	#gen1 = pc2.read_points(pc, field_names='z', skip_nans=False, uvs=[(int(newarr1[i][0]), int(newarr1[i][1]))])
	#	#gen2 = pc2.read_points(pc, field_names='z', skip_nans=False, uvs=[(int(newarr[i][0]), int(newarr[i][1]))])
	#	point1 = [newarr1[i][0],newarr1[i][1],depth_image[int(newarr1[i][1]),int(newarr1[i][0])]]
	#	point2 = [newarr[i][0],newarr[i][1],depth_image[int(newarr[i][1]),int(newarr[i][0])]]
	#	distance_btw = math.sqrt(math.pow(point1[0] - point2[0], 2) + math.pow(point1[1] - point2[1],2) + math.pow(point1[2] - point2[2], 2))
		#print(distance_btw)
	
	try:
		for i in range(17):
			#gen = pc2.read_points(self.pc, field_names='z', skip_nans=False, uvs=[(int(newarr[i][0]), int(newarr[i][1]))])
			if math.isnan(depth_image[int(newarr[i][1]),int(newarr[i][0])]) == False:
				#gen = pc2.read_points(pc, field_names='z', skip_nans=False, uvs=[(int(newarr[i][0]), int(newarr[i][1]))])
				#print(gen)
				distance[i] = (depth_image[int(newarr[i][1])-offset,int(newarr[i][0])])
				#print(distance[i])
				human_detect = 1
				conf[i] = newarr[i][2]
		
		if prev_time != -20.0 :
			for i in range(17):
				if prev_distance[i] == float('nan') or distance[i] == float('nan'):
					velocity[i] = float('nan')
					conf[i] = float('nan')
				else:
					velocity[i] = (prev_distance[i] - distance[i])/(time_now - prev_time)	
	except Exception as e:
		distance = [float('nan') for i in range(17)]
		velocity = [float('nan') for i in range(17)]
		conf = [float('nan') for i in range(17)]
		human_detect = -1
	
	prev_time = time_now
	prev_distance = distance[:]

	my_dist.data = distance
	my_vel.data = velocity
	my_conf.data = conf
	print("++++++++++++++++++++++++++++++++++++++++++++++++++++++")
	print("Human dist",distance)
	print("Human vel",velocity)
	print("Human conf",conf)
	print("++++++++++++++++++++++++++++++++++++++++++++++++++++++")
	pub1.publish(my_dist)
	pub2.publish(my_vel)
	pub3.publish(my_conf)
	pub4.publish(human_detect)
	
        

def calculate_distance(intrin):
	global distance,newarr ,newarr1, depth_image,offset,color_intrin
	global xmin1,xmin2,xmax1,xmax2,ymin1,ymin2,ymax1,ymax2, velocity, conf
	
	color_intrin = rs.intrinsics()
	color_intrin.width = intrin.width
	color_intrin.height = intrin.height
	color_intrin.ppx = intrin.K[2]
	color_intrin.ppy = intrin.K[5]
	color_intrin.fx = intrin.K[0]
	color_intrin.fy = intrin.K[4]
	if intrin.distortion_model == 'plumb_bob':
		color_intrin.model = rs.distortion.brown_conrady
	elif intrin.distortion_model == 'equidistant':
		color_intrin.model = rs.distortion.kannala_brandt4
	color_intrin.coeffs = [i for i in intrin.D]



def boundbox(data):
	global distance1, velocity, conf, depth_image, depth_img, prev_distance1, change_in_distance1, prev_time,human_detect,offset
	global xmin1,xmin2,xmax1,xmax2,ymin1,ymin2,ymax1,ymax2,color_intrin,newarr
	
	oldarr = data.data
	xmin1 = oldarr[0]
	xmin2 = oldarr[4]
	ymin1 = oldarr[1]
	ymin2 = oldarr[5]
	xmax1 = oldarr[2]
	xmax2 = oldarr[6]
	ymax1 = oldarr[3]
	ymax2 = oldarr[7]
	
	distance_btw = []	
	for i in range(17):
	
		bx1 = int((newarr[i][0]+newarr[i][0])/2)
		by1 = int(((newarr[i][1]-offset)+(newarr[i][1]-offset))/2)
		bx2 = int((xmin2+xmax2)/2)
		by2 = int((ymin2+ymax2)/2)

		point1 = rs.rs2_deproject_pixel_to_point(color_intrin, [bx1,by1], float(depth_image[by1,bx1]))
		point2 = rs.rs2_deproject_pixel_to_point(color_intrin, [bx2,by2], float(depth_image[by2,bx2]))
		distance_btw.append(math.sqrt(math.pow(point1[0] - point2[0], 2) + math.pow(point1[1] - point2[1],2) + math.pow(point1[2] - point2[2], 2)))
		print(float(depth_image[by1,bx1]),point1[2])
	#print("Distance : ",distance_btw)
	
	dist_btw.data = distance_btw
	pub5.publish(dist_btw)




	
if __name__ == '__main__':
	
	rospy.init_node('distance_velocity_calc', anonymous=True)
	pub1 = rospy.Publisher('/distance', Float64MultiArray, queue_size = 1)
	pub2 = rospy.Publisher('/velocity', Float64MultiArray, queue_size = 1)
	pub3 = rospy.Publisher('/confidence', Float64MultiArray, queue_size = 1)
	pub4 = rospy.Publisher('/human_detection', Float64, queue_size = 1)
	pub5 = rospy.Publisher('/distance_btw', Float64MultiArray, queue_size = 1)
	rospy.Subscriber("/human_skeleton_keypoints_throttle", Float64MultiArray, keypoint)
	#rospy.Subscriber("/robot_keypoints_throttle", Float64MultiArray, robot_keypoint)
	#rospy.Subscriber("/bounding_box_throttle", Float64MultiArray, boundbox)
	rospy.Subscriber('/realsense/depth/image_rect_raw',Image,get_distance)
	#rospy.Subscriber('/realsense/depth/color/points', PointCloud2,pc_distance)
	
	#rospy.Subscriber('husky2/camera/depth/image_rect_raw',Image,get_distance)
	#rospy.Subscriber('husky2/camera/depth/camera_info',CameraInfo,calculate_distance)
	while not rospy.is_shutdown():
		rospy.spin()

