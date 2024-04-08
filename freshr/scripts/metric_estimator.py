#!/usr/bin/python

import rospy , tf
import matplotlib.pyplot as plt
import torch
import cv2
import numpy as np
import time
import rospy
import math
import pyrealsense2 as rs
from nav_msgs.msg import Odometry
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
		self.theta_data = Float64MultiArray()
		self.azimuth_data = Float64MultiArray()
		self.dist_vel_data = Float64MultiArray()
		self.color_intrin = rs.intrinsics()
		self.pd = 0
		self.human_detect = 0
		self.odom_pose_x = -10.0
		self.odom_pose_y = -10.0
		self.odom_vel_x = -10.0
		self.odom_vel_y = -10.0
		
		#for case 1
		#self.num_humans_sub = rospy.Subscriber("/number_of_humans", Float64, self.n_humans)
		
		#self.depth_image_sub = rospy.Subscriber('/camera_left/aligned_depth_to_color/image_raw', Image, self.get_distance)
		self.depth_image_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.get_distance)
		self.camera_info_sub = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info',CameraInfo, self.calculate_distance)
		#self.camera_info_sub = rospy.Subscriber('/husky/realsense/depth/camera_info',CameraInfo, self.calculate_distance)
		#self.depth_image_sub = rospy.Subscriber('/husky/realsense/depth/image_rect_raw', Image, self.get_distance)
		#self.depth_image_sub = rospy.Subscriber('/realsense/depth/image_rect_raw', Image, self.get_distance)
		#self.depth_image_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw/compressed', CompressedImage, self.get_distance1)
		#self.odom_sub = rospy.Subscriber('/husky/odometry/filtered_throttle', Odometry, self.odom_cal)
		
		self.human_skeleton_keypoint_sub = rospy.Subscriber("/human_skeleton_keypoints", Float64MultiArray, self.keypoint)
		#self.human_skeleton_keypoint_sub = rospy.Subscriber("/human_skeleton_keypoints_throttle", Float64MultiArray, self.keypoint)
		
		#for case 2
		#self.depth_data_sub = rospy.Subscriber('/realsense/depth/image_rect_raw', Image, self.get_distance)
		#self.camera_info_sub = rospy.Subscriber('/husky/realsense/depth/camera_info',CameraInfo, self.calculate_distance)
		#self.box_data_sub = rospy.Subscriber("/box_data_array_throttle", Float64MultiArray, self.boundbox)	
		
		#For case 1
		#self.pub1 = rospy.Publisher('/distance', Float64MultiArray, queue_size = queue_size)
		#self.pub2 = rospy.Publisher('/velocity', Float64MultiArray, queue_size = queue_size)
		#self.pub3 = rospy.Publisher('/confidence', Float64MultiArray, queue_size = queue_size)
		#self.pub4 = rospy.Publisher('/degree_angle_of_humans', Float64MultiArray, queue_size = queue_size)
		#self.pub5 = rospy.Publisher('/azimuth_angle_of_humans', Float64MultiArray, queue_size = queue_size)
		#self.pub6 = rospy.Publisher('/number_of_humans', Float64, queue_size = queue_size)
		self.pub7 = rospy.Publisher('/dist_vel_conf_data', Float64MultiArray, queue_size = queue_size)
		
		#for case 2
		#self.pub5 = rospy.Publisher('/dist_vel_btw', Float64MultiArray, queue_size = 1)
		#self.pub6 = rospy.Publisher('/velocity_btw', Float64, queue_size = 1)
		#self.pub4 = rospy.Publisher('/degree_angle_of_humans', Float64MultiArray, queue_size = queue_size)
		#self.pub3 = rospy.Publisher('/azimuth_angle_of_humans', Float64MultiArray, queue_size = queue_size)


	def get_distance(self, img):
		bridge=CvBridge()
		self.depth_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
		#print("10")
		#print(self.depth_image)
	

	def odom_cal(self,data):

		self.odom_pose_x = data.pose.pose.position.x
		#print(self.odom_pose_x)
		self.odom_pose_y = data.pose.pose.position.y
		
		orientation_q = data.pose.pose.orientation
		quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

		euler = tf.transformations.euler_from_quaternion(quaternion)

		theta = euler[2]
		
		self.odom_vel_x = (data.twist.twist.linear.x) * math.cos((theta))#*22)/(180*7))
		self.odom_vel_y = (data.twist.twist.linear.x) * math.sin((theta))#*22)/(180*7))
		
	
	def keypoint(self, data):
		try:
			self.publishing_data = data.data
			newarr = (np.array_split(self.publishing_data, (int(len(self.publishing_data)/58))))
			time_now = rospy.get_rostime().to_sec()
			
			for nh in range((int(len(self.publishing_data)/58))):
				newarr[nh] = newarr[nh][7:]
			
			flat_list = [item for sublist in newarr for item in sublist]
					
			## BASED ON THE HUMAN DETECTIONS
			array_size = (int(len(self.publishing_data)/58))*17
			center_x = [float('nan') for i in range(array_size)]
			center_y = [float('nan') for i in range(array_size)]
			keypoint_array = (np.array_split(flat_list, array_size))
			distance_new = [float('nan') for i in range(array_size)]
			conf_new = [float('nan') for i in range(array_size)]
			velocity_new = [float('nan') for i in range(array_size)]
			##
			
			try:
				for i in range(array_size):
				
					if keypoint_array[i][2] > self.conf_thr:
						center_x[i] = keypoint_array[i][0]
						center_y[i] = keypoint_array[i][1]
				
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
						center_x[i] = np.nan
						center_y[i] = np.nan
			
				if self.prev_time != -20.0 and len(self.prev_distance)==len(distance_new):
					for i in range(array_size):
						if math.isnan(self.prev_distance[i]) == True or math.isnan(distance_new[i]) == True : #self.distance[i] == float('nan'):
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
				print("next here  -->  ", e)
				distance_new = [float('nan') for i in range(array_size)]
				velocity_new = [float('nan') for i in range(array_size)]
				conf_new = [float('nan') for i in range(array_size)]
		
			#print(self.prev_distance)
			self.prev_time = time_now
			self.prev_distance = distance_new[:]
			#print(distance_new)
			
			centroid_x = [0.0 for i in range((int(len(self.publishing_data)/58)))]
			centroid_y = [0.0 for i in range((int(len(self.publishing_data)/58)))]
			azi_angle = [float('nan') for i in range((int(len(self.publishing_data)/58)))]
			theta = [float('nan') for i in range((int(len(self.publishing_data)/58)))]
			#print("++++++++++++++++++++++++++++++++++++++++++++++++++++++")
			#print("numer of humans  ",(int(len(self.publishing_data)/58)))
			#print("center x array ",center_x)
			#print("center y array ",center_y)
			#print("centeroid x array ",centroid_x)
			#print("centeroid y array ",centroid_y)
			#print("++++++++++++++++++++++++++++++++++++++++++++++++++++++")
			
			point1 = []
			for i in range((int(len(self.publishing_data)/58))):
				count = 0
				for j in range((i*17),(i*17)+17):
					if math.isnan(center_x[j]) == False:
						count = count+1
						centroid_x[i] = centroid_x[i] + center_x[j]
						centroid_y[i] = centroid_y[i] + center_y[j]
				
				
				if count != 0:
					
					cx = centroid_x[i]/count
					cy = centroid_y[i]/count
					
					#point1 = rs.rs2_deproject_pixel_to_point(self.color_intrin, [int(cx+20),int(cy-19)], float(self.depth_image[int(cy-19),int(cx+20)]))
					#theta[i] = (69/640)*abs(cx-640)
					
					#USING CAMERA INFO
					X = (cx - self.color_intrin.ppx) * (self.depth_image[int(cy),int(cx)] / self.color_intrin.fx)
					
					azi_angle[i] = np.arctan(X / self.depth_image[int(cy),int(cx)])
					
					theta[i] = np.degrees(azi_angle[i])
					
					
					if (theta[i] < 0.0):
						theta[i] += 360.0
			
			
			####################################################################	
			##########  Code for human Velocity not complete   #################
			####################################################################
			'''	
			prh_x = self.odom_pose_x - point1[2]	
			prh_y = self.odom_pose_y - point1[0]	
			print("human xyz ", point1[0],point1[1],point1[2])
			avg_dist = 0.0 # sum(distance_new)/len(distance_new)
			length = 0
			
			avg_vel = 0.0 
			length1 = 0
			
			for i in range(array_size):
				if math.isnan(distance_new[i])==False:
					length = length + 1
					avg_dist = avg_dist + distance_new[i]
			
			avg_dist = avg_dist/length
			print("avg_dist", avg_dist)
			for i in range(array_size):
				if math.isnan(velocity_new[i])==False:
					length1 = length1 + 1
					avg_vel = avg_vel + velocity_new[i]
			
			avg_vel = avg_vel/length1
			print("avg_vel", avg_vel)
			human_vel_x = ((avg_dist*avg_vel)/prh_x) - self.odom_vel_x
			human_vel_y = ((avg_dist*avg_vel)/prh_y) - self.odom_vel_y
			print("human velocity  ",human_vel_x,human_vel_y)
			'''
		
			####################################################################	
			##########  Code for human Velocity not complete   #################
			####################################################################
		
		
		except Exception as e:
			array_size = (int(len(self.publishing_data)/58))*17
			print("next  -->  ", e)
			distance_new = [float('nan') for i in range(array_size)]
			velocity_new = [float('nan') for i in range(array_size)]
			conf_new = [float('nan') for i in range(array_size)]
			theta = [float('nan') for i in range((int(len(self.publishing_data)/58)))]
			azi_angle = [float('nan') for i in range((int(len(self.publishing_data)/58)))]	

		all_data_arr = [[],[],[],[]]
		all_data_arr[0] = distance_new[:]
		all_data_arr[1] = velocity_new[:]
		all_data_arr[2] = conf_new[:]
		all_data_arr[3] = theta[:]
		all_data_flat = [item for sublist in all_data_arr for item in sublist]
		all_data_flat.append(len(theta))
		self.my_dist.data = all_data_flat	#distance_new
		self.my_vel.data = velocity_new
		self.my_conf.data = conf_new
		#human_detection = int(len(conf_new)/17)
		#print(all_data_flat)
		#print(len(all_data_flat))
		self.theta_data.data = theta
		self.azimuth_data.data = azi_angle
		print("++++++++++++++++++++++++++++++++++++++++++++++++++++++")
		print("Human dist",distance_new)
		print("Human vel",velocity_new)
		print("Human conf",conf_new)
		#print("numer of humans  ",human_detection)
		print("azi angle for each person",azi_angle)
		print("degree angle for each person",theta)
		print("++++++++++++++++++++++++++++++++++++++++++++++++++++++")
		self.pub7.publish(self.my_dist)
		#self.pub2.publish(self.my_vel)
		#self.pub3.publish(self.my_conf)
		#self.pub4.publish(self.theta_data)
		#self.pub5.publish(self.azimuth_data)
		#self.pub6.publish(human_detection)


	

	
	
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
		print(len(oldarr))
		if (len(oldarr)) == 8:
			xmin1 = oldarr[0]
			ymin1 = oldarr[1]
			xmax1 = oldarr[2]
			ymax1 = oldarr[3]
			xmin2 = oldarr[4]
			ymin2 = oldarr[5]
			xmax2 = oldarr[6]
			ymax2 = oldarr[7]
			
		else :
			xmin1 = oldarr[0]
			ymin1 = oldarr[1]
			xmax1 = oldarr[2]
			ymax1 = oldarr[3]
			
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
		'''
		distance_btw = float('nan')
		velocity_btw = float('nan')	
		#print(xmin1,ymin1,xmax1,ymax1,xmin2,ymin2,xmax2,ymax2)
		bx1 = int((xmin1+xmax1)/2)
		by1 = int(((int(ymin1)-int(offset))+(int(ymax1)-int(offset)))/2)
		bx2 = int((xmin2+xmax2)/2)
		by2 = int(((int(ymin2)-int(offset))+(int(ymax2)-int(offset)))/2)
		#print(float(by1),float(bx1),float(by2),float(bx2))
		print(self.depth_image[by1,bx1],self.depth_image[by2,bx2])
		point1 = rs.rs2_deproject_pixel_to_point(self.color_intrin, [bx1,by1], float(self.depth_image[by1,bx1]))
		point2 = rs.rs2_deproject_pixel_to_point(self.color_intrin, [bx2,by2], float(self.depth_image[by2,bx2]))
		distance_btw = (math.sqrt(math.pow(point1[0] - point2[0], 2) + math.pow(point1[1] - point2[1],2) + math.pow(point1[2] - point2[2], 2)))
		#print(float(depth_image[by1,bx1]),point1[2])
		if self.prev_time != -20.0 :
				velocity_btw = (self.pd - distance_btw)/(time_now - self.prev_time)
		print("Distance : ",distance_btw)#,velocity_btw)
		self.prev_time = time_now
		self.pd = distance_btw
		azi_angle = [float('nan') ,float('nan')]
		dist_vel = [float('nan') ,float('nan')]
		theta = [float('nan') ,float('nan')]
		angle_data = [((self.depth_image[int(by1),int(bx1)])),bx1,((self.depth_image[int(by2),int(bx2)])),bx2]
		azi_angle[0] = math.atan2(((self.depth_image[int(by1),int(bx1)])),(bx1-340))
		azi_angle[1] = math.atan2(((self.depth_image[int(by2),int(bx2)])),(bx2-340))
		theta[0] = azi_angle[0]*(180/math.pi)
		theta[1] = azi_angle[1]*(180/math.pi)
		if (theta[0] < 0.0):
			theta[0] += 360.0
		
		print(theta,azi_angle)
		dist_vel = [distance_btw,velocity_btw]
		self.dist_vel_data.data = dist_vel[:]
		self.theta_data.data = theta
		self.azimuth_data.data = angle_data
		#self.pub6.publish(Float64(velocity_btw))
		#self.pub5.publish(Float64(distance_btw))
		self.pub5.publish(self.dist_vel_data)
		self.pub4.publish(self.theta_data)
		self.pub3.publish(self.azimuth_data)
	
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
