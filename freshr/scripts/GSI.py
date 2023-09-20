#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64MultiArray, ByteMultiArray
from std_msgs.msg import Float64 
import numpy as np
import math
import pyrealsense2 as rs


class FRESHR_GSI:
	def __init__(self, sub_topic1: str, sub_topic2: str, sub_topic3: str, safety_topic: str, dist_min_topic: str, dist_max_topic: str, dist_avg_topic: str, dist_wtd_topic: str, vel_min_topic: str, vel_max_topic: str, vel_avg_topic: str, vel_wtd_topic: str, gsi_avg_topic: str, gsi_dist_topic: str, gsi_vel_topic: str, gsi_dist_wtd_topic: str, gsi_vel_wtd_topic: str, d_factor_topic: str, v_factor_topic: str, dist_arr_topic: str, vel_arr_topic: str, conf_arr_topic: str, desired_keypoints: str, conf_thr: str, dmax: str, dmin: str, vmax: str, amax: str, dwt: str,vwt:str , rhod: str, rhov:str, queue_size: int = 10):
		
		self.desired_keypoints = list(desired_keypoints.split(","))
		self.conf_thr = float(conf_thr)
		self.dmax = float(dmax)
		self.dmin = float(dmin)
		self.vmax = float(vmax)
		self.amax = float(amax)
		self.rt = self.vmax/self.amax
		self.velocity = []
		self.conf = []
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
		self.Likert_safety = ByteMultiArray()
		self.time_prev = 0
		self.dwt = float(dwt)
		self.vwt = float(vwt)
		self.rhod = float(rhod)
		self.rhov = float(rhov)
		
		self.n_human_subscriber = rospy.Subscriber("/number_of_humans", Float64, self.n_humans)
		self.velocity_subscriber = rospy.Subscriber('/velocity', Float64MultiArray, self.transform_callback2)
		self.confidence_subscriber = rospy.Subscriber('/confidence', Float64MultiArray, self.transform_callback3)
		self.distance_subscriber = rospy.Subscriber('/distance', Float64MultiArray, self.transform_callback1)
		
		self.pub = rospy.Publisher(safety_topic, String, queue_size = queue_size)
		self.pub31 = rospy.Publisher(gsi_avg_topic, Float64, queue_size = queue_size)
		self.pub32 = rospy.Publisher(gsi_dist_topic, Float64, queue_size = queue_size)
		self.pub33 = rospy.Publisher(gsi_vel_topic, Float64, queue_size = queue_size)
		self.pub34 = rospy.Publisher(gsi_dist_wtd_topic, Float64, queue_size = queue_size)
		self.pub35 = rospy.Publisher(gsi_vel_wtd_topic, Float64, queue_size = queue_size)
		self.pub11 = rospy.Publisher(dist_min_topic, Float64, queue_size = queue_size)
		self.pub12 = rospy.Publisher(dist_max_topic, Float64, queue_size = queue_size)
		self.pub13 = rospy.Publisher(dist_avg_topic, Float64, queue_size = queue_size)
		self.pub14 = rospy.Publisher(dist_wtd_topic, Float64, queue_size = queue_size)
		self.pub21 = rospy.Publisher(vel_min_topic, Float64, queue_size = queue_size)
		self.pub22 = rospy.Publisher(vel_max_topic, Float64, queue_size = queue_size)
		self.pub23 = rospy.Publisher(vel_avg_topic, Float64, queue_size = queue_size)
		self.pub24 = rospy.Publisher(vel_wtd_topic, Float64, queue_size = queue_size)
		self.vel_fact = rospy.Publisher(v_factor_topic, Float64, queue_size = queue_size)
		self.dist_fact = rospy.Publisher(d_factor_topic, Float64, queue_size = queue_size)
		self.dist_arr = rospy.Publisher(dist_arr_topic, Float64MultiArray, queue_size = queue_size)
		self.vel_arr = rospy.Publisher(vel_arr_topic, Float64MultiArray, queue_size = queue_size)
		self.conf_arr = rospy.Publisher(conf_arr_topic, Float64MultiArray, queue_size = queue_size)


	def n_humans(self, data):
		self.human_num = data.data
		
	def transform_callback3(self, data):		
		self.conf = data.data
		self.index = [ int(i) for i in self.desired_keypoints]
    
	def transform_callback2(self, data):
		self.velocity = data.data

	def transform_callback1(self, data):

		norm = [0 for x in range(int(self.human_num))]
		ex = 0.00000000000000000000000000000000000001
		
		if self.human_num > 0 :
			distance_array = (np.array_split(data.data, self.human_num))
			velocity_array = (np.array_split(self.velocity, self.human_num))
			confidence_array = (np.array_split(self.conf, self.human_num))
			#print(confidence_array)
			#print(self.human_num)
			new_index = [[] for x in range(int(self.human_num))]
			dd1 = [[] for x in range(int(self.human_num))]
			vv1 = [[] for x in range(int(self.human_num))]
			cc1 = [[] for x in range(int(self.human_num))]
			dd2 = [[] for x in range(int(self.human_num))]
			vv2 = [[] for x in range(int(self.human_num))]
			cc2 = [[] for x in range(int(self.human_num))]
			keep_index1 = [[] for x in range(int(self.human_num))]
			dd3 = [[] for x in range(int(self.human_num))]
			vv3 = [[] for x in range(int(self.human_num))]
			cc3 = [[] for x in range(int(self.human_num))]
			keep_index2 = [[] for x in range(int(self.human_num))]
				
			for run in range(int(self.human_num)):
				for li in self.index:
					if float(confidence_array[run][li]) > self.conf_thr:
						new_index[run].append(li)
				
				for i in new_index[run]:
					dd1[run].append(distance_array[run][i])
					vv1[run].append(velocity_array[run][i])
					cc1[run].append(confidence_array[run][i])
			
				
				keep_index1[run] = [i for i,xd in enumerate(dd1[run]) if math.isnan(xd) == False]
				for i in keep_index1[run]:
					dd2[run].append(dd1[run][i])
					vv2[run].append(vv1[run][i])
					cc2[run].append(cc1[run][i])
			
				
				keep_index2[run] = [i for i,xv in enumerate(vv2[run]) if math.isnan(xv) == False]
				for i in keep_index2[run]:
					dd3[run].append(dd2[run][i])
					vv3[run].append(vv2[run][i])
					cc3[run].append(cc2[run][i])

			self.frame_distance = dd3[:]
			self.frame_velocity = vv3[:]
			self.frame_conf = cc3[:]
			
			self.frame_dist_wtd = [float(0) for x in range(int(self.human_num))]
			self.frame_dist_min = [float(0) for x in range(int(self.human_num))]
			self.frame_dist_max = [float(0) for x in range(int(self.human_num))]
			self.frame_vel_wtd = [float(0) for x in range(int(self.human_num))]
			self.frame_vel_min = [float(0) for x in range(int(self.human_num))]
			self.frame_vel_max = [float(0) for x in range(int(self.human_num))]
			self.frame_dist_avg = [float(0) for x in range(int(self.human_num))]
			self.frame_vel_avg = [float(0) for x in range(int(self.human_num))]
			self.d = [float(0) for x in range(int(self.human_num))]
			self.v = [float(0) for x in range(int(self.human_num))]
			self.os_dist = [float(0) for x in range(int(self.human_num))]
			self.os_vel = [float(0) for x in range(int(self.human_num))]
			self.os_dist_wtd = [float(0) for x in range(int(self.human_num))]
			self.os_vel_wtd = [float(0) for x in range(int(self.human_num))]
			self.os_avg = [float(0) for x in range(int(self.human_num))]
			self.safety = ["System Starting" for x in range(int(self.human_num))]

			
			for nh in range(int(self.human_num)):
				#print(nh)
				norm[nh] = sum(self.frame_conf[nh])
				#print("norm :",norm)
			
				if len(self.frame_distance[nh]) != 0 :
					self.frame_dist_avg[nh] = sum(self.frame_distance[nh])/len(self.frame_distance[nh])
				if len(self.frame_velocity[nh]) != 0 :
					self.frame_vel_avg[nh] = sum(self.frame_velocity[nh])/len(self.frame_velocity[nh])
			
				if norm[nh] != 0:
					for i in range(len(self.frame_distance[nh])):
						self.frame_dist_wtd[nh] = self.frame_dist_wtd[nh] + ((self.frame_conf[nh][i]*self.frame_distance[nh][i])/norm[nh])

					for i in range(len(self.frame_velocity[nh])):
						self.frame_vel_wtd[nh] = self.frame_vel_wtd[nh] + ((self.frame_conf[nh][i]*self.frame_velocity[nh][i])/norm[nh])

				for i in range(len(self.frame_distance[nh])):
					if norm[nh] != 0:
						self.d[nh] = self.d[nh] +(self.frame_conf[nh][i]/norm[nh])*((self.frame_distance[nh][i]-self.dmin)/(self.dmax-self.dmin))
					else:
						self.d[nh] = float('nan')
				if self.d[nh]>1 :
					self.d[nh] = 1
				elif self.d[nh]<0 :
					self.d[nh] = 0

				time_now = rospy.get_rostime().to_sec()

				for i in range(len(self.frame_velocity[nh])):
					if norm[nh] == 0.0:
						self.v[nh] = float('nan')
					elif self.frame_distance[nh][i] != float('nan') or self.frame_velocity[nh][i] != float('nan'):
						allowedv = (self.frame_distance[nh][i])/self.rt
						self.v[nh] = self.v[nh] + (self.frame_conf[nh][i]/(norm[nh]))*((allowedv - self.frame_velocity[nh][i])/(allowedv+ex))

				self.frame_dist_min[nh] = min(self.frame_distance[nh], default=float('nan'))
				self.frame_dist_max[nh] = max(self.frame_distance[nh], default=float('nan'))
				self.frame_vel_min[nh] = min(self.frame_velocity[nh], default=float('nan'))
				self.frame_vel_max[nh] = max(self.frame_velocity[nh], default=float('nan'))

				if self.v[nh]<0:
					self.v[nh] = 0
				elif self.v[nh]>1:
					self.v[nh] = 1

				if len(self.frame_distance[nh]) == 0 or len(self.frame_velocity[nh]) == 0:
					self.d[nh] = float('nan')
					self.v[nh] = float('nan')

				if self.d[nh] == float('nan') or self.v[nh] == float('nan'):
					self.os_avg[nh] = float('nan')
				else:
					self.os_avg[nh] = float(((self.dwt)*math.pow(self.d[nh],self.rhod))+((self.vwt)*math.pow(self.v[nh],self.rhov)))



				if len(self.frame_distance[nh]) == 0 or self.os_avg[nh] == float('nan'):
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

			self.safety=np.array(self.safety)
			self.d=np.array(self.d,float)
			self.v=np.array(self.v,float)
			self.os_avg=np.array(self.os_avg,float)
			self.os_dist=np.array(self.os_dist,float)
			self.os_vel=np.array(self.os_vel,float)
			self.os_dist_wtd=np.array(self.os_dist_wtd,float)
			self.os_vel_wtd=np.array(self.os_vel_wtd,float)
			self.frame_dist_min=np.array(self.frame_dist_min,float)
			self.frame_dist_max=np.array(self.frame_dist_max,float)
			self.frame_vel_min=np.array(self.frame_vel_min,float)
			self.frame_vel_max=np.array(self.frame_vel_max,float)
			self.frame_dist_avg=np.array(self.frame_dist_avg,float)
			self.frame_dist_wtd=np.array(self.frame_dist_wtd,float)
			self.frame_vel_avg=np.array(self.frame_vel_avg,float)
			self.frame_vel_wtd=np.array(self.frame_vel_wtd,float)
			self.frame_conf=np.array(self.frame_conf,float)
			self.frame_distance=np.array(self.frame_velocity,float)
			self.frame_velocity=np.array(self.frame_distance,float)
			#print(self.safety)
			#print(type(self.safety))
			print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
			print('current distance :',self.frame_dist_avg,'m distance factor :',self.d)
			#print('confidence :',self.frame_conf)
			print('current velocity :',self.frame_vel_avg,'m/s velocity factor :',self.v)
			print('GSI :',self.os_avg)
			#print('avg vs wtd :',self.frame_dist_avg,self.frame_dist_wtd)
			print(self.safety)
			print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
			
			self.time_prev = time_now
			self.frm_dist.data = self.frame_distance
			self.frm_vel.data = self.frame_velocity
			self.frm_conf.data = self.frame_conf
			#self.Likert_safety.data = self.safety
			self.pub.publish(self.safety)
			self.dist_arr.publish(self.frm_dist)
			self.vel_arr.publish(self.frm_vel)
			self.conf_arr.publish(self.frm_conf)
			self.dist_fact.publish(Float64(self.d))
			self.vel_fact.publish(Float64(self.v))
			self.pub31.publish(Float64(self.os_avg))
			self.pub32.publish(Float64(self.os_dist))
			self.pub33.publish(Float64(self.os_vel))
			self.pub34.publish(Float64(self.os_dist_wtd))
			self.pub35.publish(Float64(self.os_vel_wtd))
			self.pub11.publish(Float64(self.frame_dist_min))
			self.pub12.publish(Float64(self.frame_dist_max))
			self.pub13.publish(Float64(self.frame_dist_avg))
			self.pub14.publish(Float64(self.frame_dist_wtd))
			self.pub21.publish(Float64(self.frame_vel_min))
			self.pub22.publish(Float64(self.frame_vel_max))
			self.pub23.publish(Float64(self.frame_vel_avg))
			self.pub24.publish(Float64(self.frame_vel_wtd))
    
		else:
			print("********** NO HUMAN DETECTED **********")      
        
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
    d_factor_topic = rospy.get_param(ns + "d_factor_topic")
    v_factor_topic = rospy.get_param(ns + "v_factor_topic")
    dist_arr_topic = rospy.get_param(ns + "dist_arr_topic")
    vel_arr_topic = rospy.get_param(ns + "vel_arr_topic")
    conf_arr_topic = rospy.get_param(ns + "conf_arr_topic")
    desired_keypoints = rospy.get_param(ns + "desired_keypoints")
    conf_thr = rospy.get_param(ns + "conf_thr")
    dmax = rospy.get_param(ns + "dmax")
    dmin = rospy.get_param(ns + "dmin")
    vmax = rospy.get_param(ns + "vmax")
    amax = rospy.get_param(ns + "amax")
    dwt = rospy.get_param(ns + "dwt")
    vwt = rospy.get_param(ns + "vwt")
    rhod = rospy.get_param(ns + "rhod")
    rhov = rospy.get_param(ns + "rhov")
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
        d_factor_topic=d_factor_topic,
        v_factor_topic=v_factor_topic,
        dist_arr_topic=dist_arr_topic,
        vel_arr_topic=vel_arr_topic,
        conf_arr_topic=conf_arr_topic,
        desired_keypoints=desired_keypoints,
        conf_thr=conf_thr,
        dmax=dmax,
        dmin=dmin,
        vmax=vmax,
        amax=amax,
        dwt=dwt,
        vwt=vwt,
        rhod=rhod,
        rhov=rhov,
        queue_size = queue_size
    )

    rospy.spin()
