#!/usr/bin/python

import rospy
import matplotlib.pyplot as plt
import torch
import cv2
import numpy as np
import time
import rospy
import math
import pyrealsense2 as rs
from std_msgs.msg import String, Float64,Int32MultiArray, Float64MultiArray
from sensor_msgs.msg import Image, PointCloud2, CameraInfo, CompressedImage
from cv_bridge import CvBridge
import sensor_msgs.point_cloud2 as pc2

class METRIC_ESTIMATOR:
	def __init__(self, conf_thr: str, pc: str, offset: str, prev_time: str, queue_size: int = 10):

		self.offset = int(offset)
		self.pc = float(pc), CompressedImage
		self.conf_thr = float(conf_thr)
		self.depth_image = 0
		self.prev_time = float(prev_time)
		self.dist_btw = Float64MultiArray()
		self.dist1 = Float64MultiArray()
		self.my_dist = Float64MultiArray()
		self.my_vel = Float64MultiArray()
		self.my_conf = Float64MultiArray()
		self.color_intrin = rs.intrinsics()
		self.pd = 0
		
		#for case 1
		#self.num_humans_sub = rospy.Subscriber("/number_of_humans", Float64, self.n_humans)
		
		#self.depth_image_sub = rospy.Subscriber('/camera/image', Image, self.get_distance)
		#self.depth_image_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.get_distance)
		#self.depth_image_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw/compressed', CompressedImage, self.get_distance)
		
		#self.human_skeleton_keypoint_sub = rospy.Subscriber("/human_skeleton_keypoints", Float64MultiArray, self.keypoint)
		
		#for case 2
		self.depth_data_sub = rospy.Subscriber('/husky2/realsense/depth/image_rect_raw', Image, self.get_distance)
		self.camera_info_sub = rospy.Subscriber('/husky2/realsense/depth/camera_info',CameraInfo, self.calculate_distance)
		self.box_data_sub = rospy.Subscriber("/box_data_array_throttle", Float64MultiArray, self.boundbox)	
		
		#For case 1
		#self.pub1 = rospy.Publisher('/distance', Float64MultiArray, queue_size = queue_size)
		#self.pub2 = rospy.Publisher('/velocity', Float64MultiArray, queue_size = queue_size)
		#self.pub3 = rospy.Publisher('/confidence', Float64MultiArray, queue_size = queue_size)
		#self.pub4 = rospy.Publisher('/human_detection', Float64, queue_size = queue_size)
		
		#for case 2
		self.pub5 = rospy.Publisher('/distance_btw', Float64, queue_size = 1)
		self.pub6 = rospy.Publisher('/velocity_btw', Float64, queue_size = 1)


	def get_distance(self, img):
		bridge=CvBridge()
		self.depth_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
		

	# Functions for case 1 (Task robot view)	
	def n_humans(self, data):
		self.human_detect = int(data.data)
    
	def keypoint(self, data):	
		self.publishing_data = data.data
		newarr = (np.array_split(self.publishing_data, self.human_detect))
		time_now = rospy.get_rostime().to_sec()
		
		for nh in range(self.human_detect):
			newarr[nh] = newarr[nh][7:]
		
		flat_list = [item for sublist in newarr for item in sublist]
				
		## BASED ON THE HUMAN DETECTIONS
		array_size = self.human_detect*17
		keypoint_array = (np.array_split(flat_list, array_size))
		distance_new = [float('nan') for i in range(array_size)]
		conf_new = [float('nan') for i in range(array_size)]
		velocity_new = [float('nan') for i in range(array_size)]
		##
		
		try:
			for i in range(array_size):
				if keypoint_array[i][2] > self.conf_thr:
					if math.isnan(self.depth_image[int(keypoint_array[i][1]),int(keypoint_array[i][0])]) == False:
						#self.distance[i] = (self.depth_image[int(keypoint_array[i][1]),int(keypoint_array[i][0])])/1000 #-self.offset
						distance_new[i] = (self.depth_image[int(keypoint_array[i][1]),int(keypoint_array[i][0])])/1000
						#self.conf[i] = keypoint_array[i][2]
						conf_new[i] = keypoint_array[i][2]
						#print(self.distance)
					else:
						#print(i)
						#self.distance[i] = float('nan')
						distance_new[i] = float('nan')
						#self.conf[i] = float('nan')
						conf_new[i] = float('nan')
						
				else :
					conf_new[i] = float('nan')
		
			if self.prev_time != -20.0 and len(self.prev_distance)==len(distance_new):
				for i in range(array_size):
					if self.prev_distance[i] == float('nan') or distance_new[i] == float('nan') : #self.distance[i] == float('nan'):
						velocity_new[i] = float('nan')
						#self.conf[i] = float('nan')
						conf_new[i] = float('nan')
					else:
						#self.velocity[i] = (self.prev_distance[i] - self.distance[i])/(time_now - self.prev_time)
						velocity_new[i] = (self.prev_distance[i] - distance_new[i])/(time_now - self.prev_time)
			else:
				velocity_new = [float('nan') for i in range(array_size)]
				self.prev_distance = distance_new[:]
				
				
		except Exception as e:
			print("next  -->  ", e)
			distance_new = [float('nan') for i in range(array_size)]
			velocity_new = [float('nan') for i in range(array_size)]
			conf_new = [float('nan') for i in range(array_size)]
		
		print(self.prev_distance)
		self.prev_time = time_now
		self.prev_distance = distance_new[:]
		print(distance_new)



		self.my_dist.data = distance_new
		self.my_vel.data = velocity_new
		self.my_conf.data = conf_new
		print("++++++++++++++++++++++++++++++++++++++++++++++++++++++")
		print("Human dist",distance_new)
		print("Human vel",velocity_new)
		print("Human conf",conf_new)
		print("++++++++++++++++++++++++++++++++++++++++++++++++++++++")
		self.pub1.publish(self.my_dist)
		self.pub2.publish(self.my_vel)
		self.pub3.publish(self.my_conf)


	#Function for case 2 Observing robot view 	
	def calculate_distance(self, intrin):

		self.color_intrin.width = intrin.width
		self.color_intrin.height = intrin.height
		self.color_intrin.ppx = intrin.K[2]
		self.color_intrin.ppy = intrin.K[5]
		self.color_intrin.fx = intrin.K[0]
		self.color_intrin.fy = intrin.K[4]
		if intrin.distortion_model == 'plumb_bob':
			self.color_intrin.model = rs.distortion.brown_conrady
		elif intrin.distortion_model == 'equidistant':
			self.color_intrin.model = rs.distortion.kannala_brandt4
		self.color_intrin.coeffs = [i for i in intrin.D]



	def boundbox(self, data):

		time_now = rospy.get_rostime().to_sec()
		oldarr = data.data
		#print(oldarr)
		'''
		n_object = len(oldarr)/5
		print(n_object)
		box_data = (np.array_split(oldarr, n_object))
		print(box_data)
		if n_object == 2:
			xmin1 = oldarr[0]
			xmin2 = oldarr[5]
			ymin1 = oldarr[1]
			ymin2 = oldarr[6]
			xmax1 = oldarr[2]
			xmax2 = oldarr[7]
			ymax1 = oldarr[3]
			ymax2 = oldarr[8]
			
		else:
		'''
		if int(oldarr[4]) == 9:
			xmin1 = oldarr[5]
			xmax1 = oldarr[6]
			ymin1 = oldarr[7]
			ymax1 = oldarr[8]
			xmin2 = oldarr[10]
			xmax2 = oldarr[11]
			ymin2 = oldarr[12]
			ymax2 = oldarr[13]
			
		elif int(oldarr[9]) == 9:
			xmin1 = oldarr[0]
			xmax1 = oldarr[1]
			ymin1 = oldarr[2]
			ymax1 = oldarr[3]
			xmin2 = oldarr[10]
			xmax2 = oldarr[11]
			ymin2 = oldarr[12]
			ymax2 = oldarr[13]
			
		else:
			xmin1 = oldarr[0]
			xmax1 = oldarr[1]
			ymin1 = oldarr[2]
			ymax1 = oldarr[3]
			xmin2 = oldarr[5]
			xmax2 = oldarr[6]
			ymin2 = oldarr[7]
			ymax2 = oldarr[8]
		
		distance_btw = 0.0	
			
		bx1 = int((xmin1+xmax1)/2)
		by1 = int(((int(ymin1)-int(offset))+(int(ymax1)-int(offset)))/2)
		bx2 = int((xmin2+xmax2)/2)
		by2 = int((ymin2+ymax2)/2)
		print(float(by1),float(bx1))
		print(self.depth_image[by1,bx1])
		point1 = rs.rs2_deproject_pixel_to_point(self.color_intrin, [bx1,by1], float(self.depth_image[by1,bx1]))
		point2 = rs.rs2_deproject_pixel_to_point(self.color_intrin, [bx2,by2], float(self.depth_image[by2,bx2]))
		distance_btw = (math.sqrt(math.pow(point1[0] - point2[0], 2) + math.pow(point1[1] - point2[1],2) + math.pow(point1[2] - point2[2], 2)))
		#print(float(depth_image[by1,bx1]),point1[2])
		if self.prev_time != -20.0 :
				velocity_btw = (self.pd - distance_btw)/(time_now - self.prev_time)
		print("Distance : ",distance_btw)#,velocity_btw)
		self.prev_time = time_now
		self.pd = distance_btw
		
		self.pub6.publish(Float64(velocity_btw))
		self.pub5.publish(Float64(distance_btw))
	
if __name__ == '__main__':
	
	rospy.init_node('metric_estimator')
	
	ns = rospy.get_name() + "/"
	
	conf_thr = rospy.get_param(ns + "conf_thr")
	offset = rospy.get_param(ns + "offset")
	pc = rospy.get_param(ns + "pc")
	prev_time = rospy.get_param(ns + "prev_time")
	queue_size = rospy.get_param(ns + "queue_size")
	
	estimator = METRIC_ESTIMATOR(conf_thr=conf_thr,pc=pc,offset=offset,prev_time=prev_time,queue_size=queue_size)
	rospy.spin()
