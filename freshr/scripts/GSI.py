#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float64MultiArray, ByteMultiArray
from std_msgs.msg import Float64 
import numpy as np
import math
import pyrealsense2 as rs

class FRESHR_GSI:
	def __init__(self, sub_topic1: str, sub_topic2: str, sub_topic3: str, safety_topic: str, dist_min_topic: str, dist_max_topic: str, dist_avg_topic: str, dist_wtd_topic: str, vel_min_topic: str, vel_max_topic: str, vel_avg_topic: str, vel_wtd_topic: str, gsi_avg_topic: str, gsi_dist_topic: str, gsi_vel_topic: str, gsi_dist_wtd_topic: str, gsi_vel_wtd_topic: str, dist_arr_topic: str, vel_arr_topic: str, conf_arr_topic: str, overall_gsi_topic: str, desired_keypoints: str, conf_thr: str, dmax: str, dmin: str, vmax: str, amax: str, rho: str, queue_size: int = 10):
		
		self.desired_keypoints = list(desired_keypoints.split(","))
		self.conf_thr = float(conf_thr)
		self.dmax = float(dmax)
		self.dmin = float(dmin)
		self.vmax = float(vmax)
		self.amax = float(amax)
		self.rt = self.vmax/self.amax
		self.velocity = []
		self.conf = []
		self.distance = []
		self.index = []
		self.frame_distance = []
		self.frame_velocity = []
		self.frame_conf = []
		self.safety_value = float('nan')
		self.safety = "System Starting"
		self.prev_d = -1.0
		self.frame_dist_avg = 0
		self.frame_vel_avg = 0
		self.frame_dist_wtd = 0
		self.frame_vel_wtd = 0
		self.frm_dist = Float64MultiArray()
		self.frm_vel = Float64MultiArray()
		self.frm_conf = Float64MultiArray()
		self.bounding_box = Float64MultiArray()
		self.Likert_safety = ByteMultiArray()
		self.time_prev = 0
		self.rho = float(rho)

		
		#self.safety=np.array(self.safety)
		self.d_data = Float64MultiArray()
		self.v_data = Float64MultiArray()
		self.os_avg_data = Float64MultiArray()
		self.os_dist_data = Float64MultiArray()
		self.os_vel_data = Float64MultiArray()
		self.os_dist_wtd_data = Float64MultiArray()
		self.os_vel_wtd_data = Float64MultiArray()
		self.frame_dist_min_data = Float64MultiArray()
		self.frame_dist_max_data = Float64MultiArray()
		self.frame_vel_min_data = Float64MultiArray()
		self.frame_vel_max_data = Float64MultiArray()
		self.frame_dist_avg_data = Float64MultiArray()
		self.frame_dist_wtd_data = Float64MultiArray()
		self.frame_vel_avg_data = Float64MultiArray()
		self.frame_vel_wtd_data = Float64MultiArray()
		
		#Subscribing FOR CASE 1 ONLY (Task Robot Viewpoint)
		self.all_data_subscriber = rospy.Subscriber('/dist_vel_conf_data', Float64MultiArray, self.transform_callback1)
		
		# Subscribing FOR CASE 2 ONLY (Observer Viewpoint)
		#self.all_data_subscriber = rospy.Subscriber('/dist_vel_btw', Float64MultiArray, self.transform_callback4)
		
		
		# Publishers
		
		self.pub = rospy.Publisher(safety_topic, String, queue_size = queue_size)
		self.pub31 = rospy.Publisher(gsi_avg_topic, Float64MultiArray, queue_size = queue_size)
		self.pub32 = rospy.Publisher(gsi_dist_topic, Float64MultiArray, queue_size = queue_size)
		self.pub33 = rospy.Publisher(gsi_vel_topic, Float64MultiArray, queue_size = queue_size)
		self.pub34 = rospy.Publisher(gsi_dist_wtd_topic, Float64MultiArray, queue_size = queue_size)
		self.pub35 = rospy.Publisher(gsi_vel_wtd_topic, Float64MultiArray, queue_size = queue_size)
		self.pub11 = rospy.Publisher(dist_min_topic, Float64MultiArray, queue_size = queue_size)
		self.pub12 = rospy.Publisher(dist_max_topic, Float64MultiArray, queue_size = queue_size)
		self.pub13 = rospy.Publisher(dist_avg_topic, Float64MultiArray, queue_size = queue_size)
		self.pub14 = rospy.Publisher(dist_wtd_topic, Float64MultiArray, queue_size = queue_size)
		self.pub21 = rospy.Publisher(vel_min_topic, Float64MultiArray, queue_size = queue_size)
		self.pub22 = rospy.Publisher(vel_max_topic, Float64MultiArray, queue_size = queue_size)
		self.pub23 = rospy.Publisher(vel_avg_topic, Float64MultiArray, queue_size = queue_size)
		self.pub24 = rospy.Publisher(vel_wtd_topic, Float64MultiArray, queue_size = queue_size)
		self.dist_arr = rospy.Publisher(dist_arr_topic, Float64MultiArray, queue_size = queue_size)
		self.vel_arr = rospy.Publisher(vel_arr_topic, Float64MultiArray, queue_size = queue_size)
		self.conf_arr = rospy.Publisher(conf_arr_topic, Float64MultiArray, queue_size = queue_size)
		self.overall_gsi =  rospy.Publisher(overall_gsi_topic, Float64, queue_size = queue_size)


	'''
	def transform_callback4(self, data):

		ex = 0.00000000000000000000000000000000000001
		alldata_array = (np.array_split(data.data, 2))
		distance = alldata_array[0]
		velocity = alldata_array[1]
		self.d = [float('nan')]
		self.v = [float('nan')]
		self.os_dist = [float('nan')]
		self.os_vel = [float('nan')]
		self.os_dist_wtd = [float('nan')]
		self.os_vel_wtd = [float('nan')]
		self.os_avg = [float('nan')]
		self.safety = ["System Starting"]

		self.d = ((distance-self.dmin)/(self.dmax-self.dmin))

		if self.d>1 :
			self.d = 1
		elif self.d<0 :
			self.d = 0

		time_now = rospy.get_rostime().to_sec()

		allowedv = (distance)/self.rt
		self.v = ((allowedv - velocity)/(allowedv+ex))

		if self.v<0:
			self.v = 0
		elif self.v>1:
			self.v = 1


		if math.isnan(float(self.d)) == True :
			self.os_avg = math.pow(self.v,self.rhov)
		
		elif math.isnan(float(self.v)) == True :
			self.os_avg = math.pow(self.d,self.rhod)
			
		elif math.isnan(float(self.d)) == True  and math.isnan(float(self.v)) == True :
			self.os_avg = float('nan')
		
		else:
			self.os_avg = float(((self.dwt)*math.pow(self.d,self.rhod))+((self.vwt)*math.pow(self.v,self.rhov)))



		if len(distance) == 0 or math.isnan(float(self.os_avg)) == True:
			self.safety = "Nothing Detected or System Unavailable"
		elif self.os_avg < 0.1 :
			self.safety = "Unsafe"
		elif self.os_avg >= 0.1 and self.os_avg < 0.28 :
			self.safety = "Mostly Unsafe"
		elif self.os_avg >= 0.28 and self.os_avg < 0.45:
			self.safety = "Slightly Unsafe"
		elif self.os_avg >= 0.45 and self.os_avg < 0.55:
			self.safety = "Neither Safe nor Unsafe (Neutral)"
		elif self.os_avg > 0.55 and self.os_avg <= 0.73 :
			self.safety = "Slightly Safe" 
		elif self.os_avg > 0.73 and self.os_avg <= 0.9 :
			self.safety = "Mostly Safe"
		elif self.os_avg > 0.9 :
			self.safety = "Safe"

		self.safety=self.safety
		self.d_data.data = [self.d]
		self.v_data.data = [self.v]
		self.os_avg_data.data = [self.os_avg]
		#self.os_dist_data.data = self.os_dist
		#self.os_vel_data.data = self.os_vel
		#self.os_dist_wtd_data.data = self.os_dist_wtd
		#self.os_vel_wtd_data.data = self.os_vel_wtd
		#self.frame_dist_min_data.data = self.frame_dist_min
		#self.frame_dist_max_data.data = self.frame_dist_max
		#self.frame_vel_min_data.data = self.frame_vel_min
		#self.frame_vel_max_data.data = self.frame_vel_max
		#self.frame_dist_avg_data.data = self.frame_dist_avg
		#self.frame_dist_wtd_data.data = self.frame_dist_wtd
		#self.frame_vel_avg_data.data = self.frame_vel_avg
		#self.frame_vel_wtd_data.data = self.frame_vel_wtd
		#self.frame_conf=np.array(self.frame_conf, dtype=object)
		self.frame_distance=distance
		self.frame_velocity=velocity
		#print(self.safety)
		#print(type(self.safety))
		print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
		print('current distance :',distance)	#,'m distance factor :',self.d)
		#print('confidence :',self.frame_conf)
		print('current velocity :',velocity)	#,'m/s velocity factor :',self.v)
		print('GSI :',gsi_o)
		#print('avg vs wtd :',self.frame_dist_avg,self.frame_dist_wtd)
		print(self.safety)
		print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
		
		self.time_prev = time_now
		self.frm_dist.data = distance
		self.frm_vel.data = velocity
		#self.frm_conf.data = self.frame_conf
		#self.Likert_safety.data = self.safety
		#self.pub.publish(self.safety)
		self.dist_arr.publish(self.frm_dist)
		self.vel_arr.publish(self.frm_vel)
		#self.conf_arr.publish(self.frm_conf)
		self.dist_fact.publish((self.d_data))
		self.vel_fact.publish((self.v_data))
		#self.pub31.publish((self.os_avg_data))
		#self.pub32.publish((self.os_dist_data))
		#self.pub33.publish((self.os_vel_data))
		#self.pub34.publish((self.os_dist_wtd_data))
		#self.pub35.publish((self.os_vel_wtd_data))
		#self.pub11.publish((self.frame_dist_min_data))
		#self.pub12.publish((self.frame_dist_max_data))
		#self.pub13.publish((self.frame_dist_avg_data))
		#self.pub14.publish((self.frame_dist_wtd_data))
		#self.pub21.publish((self.frame_vel_min_data))
		#self.pub22.publish((self.frame_vel_max_data))
		#self.pub23.publish((self.frame_vel_avg_data))
		#self.pub24.publish((self.frame_vel_wtd_data))
		#self.overall_gsi.publish((human_num))
    
		
	'''
	def transform_callback1(self, data):

		self.index = [ int(i) for i in self.desired_keypoints]
		alpha = 0.01
		human_num = int(data.data[-1:][0])
		theta = data.data[-(human_num+1):len(data.data)-1]
		yolo_data = data.data[:len(data.data)-(human_num+1)]
		norm = [0 for x in range(int(human_num))]
		ex = 0.00000000000000000000000000000000000001
		all_data_arr = (np.array_split(yolo_data, 3))
		
		if human_num > 0 :
			distance_array = (np.array_split(all_data_arr[0], human_num))
			velocity_array = (np.array_split(all_data_arr[1], human_num))
			confidence_array = (np.array_split(all_data_arr[2], human_num))

			new_index = [[] for x in range(int(human_num))]
			dd1 = [[] for x in range(int(human_num))]
			vv1 = [[] for x in range(int(human_num))]
			cc1 = [[] for x in range(int(human_num))]
			ad1 = [[] for x in range(int(human_num))]
			dd2 = [[] for x in range(int(human_num))]
			vv2 = [[] for x in range(int(human_num))]
			cc2 = [[] for x in range(int(human_num))]
			ad2 = [[] for x in range(int(human_num))]
			keep_index1 = [[] for x in range(int(human_num))]
			dd3 = [[] for x in range(int(human_num))]
			vv3 = [[] for x in range(int(human_num))]
			cc3 = [[] for x in range(int(human_num))]
			keep_index2 = [[] for x in range(int(human_num))]

			for run in range(int(human_num)):
				for li in self.index:
					if float(confidence_array[run][li]) > self.conf_thr:
						new_index[run].append(li)
				
				for i in new_index[run]:
					dd1[run].append(distance_array[run][i])
					vv1[run].append(velocity_array[run][i])
					cc1[run].append(confidence_array[run][i])
					#ad1[run].append(angle_array[run][i])
			
				
				keep_index1[run] = [i for i,xd in enumerate(dd1[run]) if math.isnan(xd) == False]
				for i in keep_index1[run]:
					dd2[run].append(dd1[run][i])
					vv2[run].append(vv1[run][i])
					cc2[run].append(cc1[run][i])
					#ad2[run].append(ad1[run][i])
			
				
				keep_index2[run] = [i for i,xv in enumerate(vv2[run]) if math.isnan(xv) == False]
				for i in keep_index2[run]:
					dd3[run].append(dd2[run][i])
					vv3[run].append(vv2[run][i])
					cc3[run].append(cc2[run][i])

			self.frame_distance = dd2[:]
			self.frame_velocity = vv2[:]
			self.frame_conf = cc2[:]
			
			self.frame_dist_wtd = [float(0) for x in range(int(human_num))]
			self.frame_dist_min = [float(0) for x in range(int(human_num))]
			self.frame_dist_max = [float(0) for x in range(int(human_num))]
			self.frame_vel_wtd = [float(0) for x in range(int(human_num))]
			self.frame_vel_min = [float(0) for x in range(int(human_num))]
			self.frame_vel_max = [float(0) for x in range(int(human_num))]
			self.frame_dist_avg = [float(0) for x in range(int(human_num))]
			self.frame_vel_avg = [float(0) for x in range(int(human_num))]
			self.d = [float(0) for x in range(int(human_num))]
			self.v = [float(0) for x in range(int(human_num))]
			self.os_dist = [float(0) for x in range(int(human_num))]
			self.os_vel = [float(0) for x in range(int(human_num))]
			self.os_dist_wtd = [float(0) for x in range(int(human_num))]
			self.os_vel_wtd = [float(0) for x in range(int(human_num))]
			self.os_avg = [float(0) for x in range(int(human_num))]
			self.safety = ["System Starting" for x in range(int(human_num))]

			
			for nh in range(int(human_num)):
				norm[nh] = sum(self.frame_conf[nh])
			
				if len(self.frame_distance[nh]) != 0 :
					self.frame_dist_avg[nh] = sum(self.frame_distance[nh])/len(self.frame_distance[nh])
				else :
					self.frame_dist_avg[nh] = float('nan')
				
				if len(self.frame_velocity[nh]) != 0 :
					self.frame_vel_avg[nh] = sum(self.frame_velocity[nh])/len(self.frame_velocity[nh])
				else:
					self.frame_vel_avg[nh] = float('nan')
			
				if norm[nh] != 0:
					for i in range(len(self.frame_distance[nh])):
						self.frame_dist_wtd[nh] = self.frame_dist_wtd[nh] + ((self.frame_conf[nh][i]*self.frame_distance[nh][i])/norm[nh])

					for i in range(len(self.frame_velocity[nh])):
						self.frame_vel_wtd[nh] = self.frame_vel_wtd[nh] + ((self.frame_conf[nh][i]*self.frame_velocity[nh][i])/norm[nh])
				else:
					self.frame_dist_wtd[nh] = float('nan')
					self.frame_vel_wtd[nh] = float('nan')

				time_now = rospy.get_rostime().to_sec()

				self.frame_dist_min[nh] = min(self.frame_distance[nh], default=float('nan'))
				self.frame_dist_max[nh] = max(self.frame_distance[nh], default=float('nan'))
				self.frame_vel_min[nh] = min(self.frame_velocity[nh], default=float('nan'))
				self.frame_vel_max[nh] = max(self.frame_velocity[nh], default=float('nan'))
				
				self.os_avg[nh] = ((self.frame_dist_wtd[nh] - (np.sign(self.frame_vel_wtd[nh])*(self.frame_vel_wtd[nh]**2/2*self.amax) + self.dmin ))/(self.dmax - self.dmin))**self.rho	#
				print(self.frame_dist_wtd[nh], np.sign(self.frame_vel_wtd[nh]), (self.frame_vel_wtd[nh]**2/2*self.amax), self.dmin, (self.dmax - self.dmin)	)

				if len(self.frame_distance[nh]) == 0 or math.isnan(float(self.os_avg[nh])) == True:
					self.safety[nh] = "Nothing Detected or System Unavailable"
				elif self.os_avg[nh] < 0.1 :
					self.safety[nh] = "Unsafe"
				elif self.os_avg[nh] >= 0.1 and self.os_avg[nh] < 0.28 :
					self.safety[nh] = "Mostly Unsafe"
				elif self.os_avg[nh] >= 0.28 and self.os_avg[nh] < 0.45:
					self.safety[nh] = "Slightly Unsafe"
				elif self.os_avg[nh] >= 0.45 and self.os_avg[nh] < 0.55:
					self.safety[nh] = "Neither Safe nor Unsafe (Neutral)"
				elif self.os_avg[nh] > 0.55 and self.os_avg[nh] <= 0.73 :
					self.safety[nh] = "Slightly Safe" 
				elif self.os_avg[nh] > 0.73 and self.os_avg[nh] <= 0.9 :
					self.safety[nh] = "Mostly Safe"
				elif self.os_avg[nh] > 0.9 :
					self.safety[nh] = "Safe"
					
			
			gsi_ang = [float('nan') for i in range(int(human_num))]
			for i in range(int(human_num)):
				gsi_ang[i] = 1 - ((1-self.os_avg[i])*math.cos((theta[i]*22)/(180*7)))
			
			newgsiangles = [x for x in gsi_ang if not math.isnan(x)]
			if len(newgsiangles) == 0:
				return np.nan
			
			gsi_o = -alpha * np.log((1 / len(newgsiangles)) * np.sum(np.exp(-np.array(newgsiangles) / alpha)))	
			collective_gsi = np.clip(gsi_o, 0, 1)
			
			
			self.safety=np.array(self.safety)
			self.os_avg_data.data = self.os_avg[:]
			self.os_dist_data.data = self.gsi_ang[:]
			self.os_vel_data.data = self.os_vel
			self.os_dist_wtd_data.data = self.os_dist_wtd
			self.os_vel_wtd_data.data = self.os_vel_wtd
			self.frame_dist_min_data.data = self.frame_dist_min
			self.frame_dist_max_data.data = self.frame_dist_max
			self.frame_vel_min_data.data = self.frame_vel_min
			self.frame_vel_max_data.data = self.frame_vel_max
			self.frame_dist_avg_data.data = self.frame_dist_avg
			#self.frame_dist_wtd_data.data = self.frame_dist_wtd
			self.frame_vel_avg_data.data = self.frame_vel_avg
			self.frame_vel_wtd_data.data = self.frame_vel_wtd
			self.frame_conf=np.array(self.frame_conf, dtype=object)
			self.frame_distance=np.array(self.frame_velocity, dtype=object)
			self.frame_velocity=np.array(self.frame_distance, dtype=object)

			print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
			print('current distance :',self.frame_dist_wtd)#,'m distance factor :',self.d)
			print('current velocity :',self.frame_vel_wtd)#,'m/s velocity factor :',self.v)
			print('GSI with bearing :',gsi_ang)
			print('Collective GSI :',collective_gsi)
			print(self.safety)
			print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
			
			self.time_prev = time_now
			self.frm_dist.data = self.frame_distance
			self.frm_vel.data = self.frame_velocity
			self.frm_conf.data = self.frame_conf
			
			record_data = [[],[],[],[],[]]
			record_data[0] = self.frame_dist_wtd
			record_data[1] = self.frame_vel_wtd
			record_data[2] = self.d
			record_data[3] = self.v
			record_data[4] = theta
			record_data_flat = [item for sublist in record_data for item in sublist]
			record_data_flat.append(human_num)
			self.frame_dist_wtd_data.data = record_data_flat
			#self.Likert_safety.data = self.safety
			#self.pub.publish(self.safety)
			#self.dist_arr.publish(self.frm_dist)
			#self.vel_arr.publish(self.frm_vel)
			#self.conf_arr.publish(self.frm_conf)
			#self.dist_fact.publish((self.d_data))
			#self.vel_fact.publish((self.v_data))
			self.pub31.publish((self.os_avg_data))
			self.pub32.publish((self.os_dist_data))
			#self.pub33.publish((self.os_vel_data))
			#self.pub34.publish((self.os_dist_wtd_data))
			#self.pub35.publish((self.os_vel_wtd_data))
			#self.pub11.publish((self.frame_dist_min_data))
			#self.pub12.publish((self.frame_dist_max_data))
			#self.pub13.publish((self.frame_dist_avg_data))
			self.pub14.publish((self.frame_dist_wtd_data))
			#self.pub21.publish((self.frame_vel_min_data))
			#self.pub22.publish((self.frame_vel_max_data))
			#self.pub23.publish((self.frame_vel_avg_data))
			#self.pub24.publish((self.frame_vel_wtd_data))
			self.overall_gsi.publish(collective_gsi)
    
		else:
			print("********** NO HUMAN DETECTED **********")
			self.overall_gsi.publish(float(1))

        


if __name__ == "__main__":
    rospy.init_node("Safety_scale")

    ns = rospy.get_name() + "/"

    dist_topic = rospy.get_param(ns + "dist_topic")
    velo_topic = rospy.get_param(ns + "velo_topic")
    conf_topic = rospy.get_param(ns + "conf_topic")
    safety_topic = rospy.get_param(ns + "safety_topic")
    dist_min_topic = rospy.get_param(ns + "dist_min_topic")
    dist_max_topic = rospy.get_param(ns + "dist_max_topic")
    dist_avg_topic = rospy.get_param(ns + "dist_avg_topic")
    dist_wtd_topic = rospy.get_param(ns + "dist_wtd_topic")
    vel_min_topic = rospy.get_param(ns + "vel_min_topic")
    vel_max_topic = rospy.get_param(ns + "vel_max_topic")
    vel_avg_topic = rospy.get_param(ns + "vel_avg_topic")
    vel_wtd_topic = rospy.get_param(ns + "vel_wtd_topic")
    gsi_avg_topic = rospy.get_param(ns + "gsi_avg_topic")
    gsi_dist_topic = rospy.get_param(ns + "gsi_dist_topic")
    gsi_vel_topic = rospy.get_param(ns + "gsi_vel_topic")
    gsi_dist_wtd_topic = rospy.get_param(ns + "gsi_dist_wtd_topic")
    gsi_vel_wtd_topic = rospy.get_param(ns + "gsi_vel_wtd_topic")
    dist_arr_topic = rospy.get_param(ns + "dist_arr_topic")
    vel_arr_topic = rospy.get_param(ns + "vel_arr_topic")
    conf_arr_topic = rospy.get_param(ns + "conf_arr_topic")
    overall_gsi_topic = rospy.get_param(ns + "overall_gsi_topic")
    desired_keypoints = rospy.get_param(ns + "desired_keypoints")
    conf_thr = rospy.get_param(ns + "conf_thr")
    dmax = rospy.get_param(ns + "dmax")
    dmin = rospy.get_param(ns + "dmin")
    vmax = rospy.get_param(ns + "vmax")
    amax = rospy.get_param(ns + "amax")
    rho = rospy.get_param(ns + "rho")
    queue_size = rospy.get_param(ns + "queue_size")


    publisher = FRESHR_GSI(
        sub_topic1=dist_topic,
        sub_topic2=velo_topic,
        sub_topic3=conf_topic,
        safety_topic=safety_topic,
        dist_min_topic=dist_min_topic,
        dist_max_topic=dist_max_topic,
        dist_avg_topic=dist_avg_topic,
        dist_wtd_topic=dist_wtd_topic,
        vel_min_topic=vel_min_topic,
        vel_max_topic=vel_max_topic,
        vel_avg_topic=vel_avg_topic,
        vel_wtd_topic=vel_wtd_topic,
        gsi_avg_topic=gsi_avg_topic,
        gsi_dist_topic=gsi_dist_topic,
        gsi_vel_topic=gsi_vel_topic,
        gsi_dist_wtd_topic=gsi_dist_wtd_topic,
        gsi_vel_wtd_topic=gsi_vel_wtd_topic,
        dist_arr_topic=dist_arr_topic,
        vel_arr_topic=vel_arr_topic,
        conf_arr_topic=conf_arr_topic,
        overall_gsi_topic = overall_gsi_topic,
        desired_keypoints=desired_keypoints,
        conf_thr=conf_thr,
        dmax=dmax,
        dmin=dmin,
        vmax=vmax,
        amax=amax,
        rho=rho,
        queue_size = queue_size
    )

    rospy.spin()
