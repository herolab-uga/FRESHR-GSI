#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64MultiArray
from std_msgs.msg import Float64 
import numpy as np
import math

global safety, frame_distance, frame_velocity, safety_value, norm, index,conf_thr, conf, velocity, frame_conf, desired_keypoints
desired_keypoints = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16] #rospy.get_param("desired_keypoints")
velocity = []
index = []
conf = []
conf_thr = rospy.get_param("conf_thr")
frame_distance = [0.0]
frame_velocity = [0.0]
frame_conf = []
safety_value = float('nan')
dmax = rospy.get_param("dmax") # Start detecting the distance once dmax is breached.
dmin = rospy.get_param("dmin") # minimum separation distance that has to be maintained during the operation.
vmax = rospy.get_param("vmax")  # maximum speed of the robot is 2 m/s.
amax = rospy.get_param("amax")  # maximum acceleration of the robot is 0.7 m/s^2.
rt = vmax/amax # rt is the reaction time of the robot. It would take 1s for the robot with max speed and max acceleration to come to complete stop.
safety = rospy.get_param("safety")
prev_d = -1.0
frame_dist_avg = 0
frame_vel_avg = 0
frame_dist_wtd = 0
frame_vel_wtd = 0
frm_dist = Float64MultiArray()
frm_vel = Float64MultiArray()
frm_conf = Float64MultiArray()

pub = rospy.Publisher('/Framework/safety', String, queue_size = 10)
pub31 = rospy.Publisher('/Framework/safety_value_avg', Float64, queue_size = 10)
pub32 = rospy.Publisher('/Framework/safety_value_dist', Float64, queue_size = 10)
pub33 = rospy.Publisher('/Framework/safety_value_vel', Float64, queue_size = 10)
pub34 = rospy.Publisher('/Framework/safety_value_dist_wtd', Float64, queue_size = 10)
pub35 = rospy.Publisher('/Framework/safety_value_vel_wtd', Float64, queue_size = 10)
pub11 = rospy.Publisher('/Framework/distance_min', Float64, queue_size = 10)
pub12 = rospy.Publisher('/Framework/distance_max', Float64, queue_size = 10)
pub13 = rospy.Publisher('/Framework/distance_avg', Float64, queue_size = 10)
pub14 = rospy.Publisher('/Framework/distance_wtd', Float64, queue_size = 10)
pub21 = rospy.Publisher('/Framework/velocity_min', Float64, queue_size = 10)
pub22 = rospy.Publisher('/Framework/velocity_max', Float64, queue_size = 10)
pub23 = rospy.Publisher('/Framework/velocity_avg', Float64, queue_size = 10)
pub24 = rospy.Publisher('/Framework/velocity_wtd', Float64, queue_size = 10)
vel_fact = rospy.Publisher('/Framework/velocity_factor', Float64, queue_size = 10)
dist_fact = rospy.Publisher('/Framework/distance_factor', Float64, queue_size = 10)
dist_arr = rospy.Publisher('/Framework/distance_array', Float64MultiArray, queue_size = 10)
vel_arr = rospy.Publisher('/Framework/velocity_array', Float64MultiArray, queue_size = 10)
conf_arr = rospy.Publisher('/Framework/confidence_array', Float64MultiArray, queue_size = 10)
time_prev = 0


def likert_scale(lp):
	if lp == 1:
		return "Unsafe"
	elif lp == 2:
		return "Mostly Unsafe"
	elif lp == 3:
		return "Slightly Unsafe"
	elif lp == 4:
		return "Neither Safe nor Unsafe (Neutral)"
	elif lp == 5:
		return "Slightly Safe"
	elif lp == 6:
		return "Mostly Safe"
	elif lp == 7:
		return "Safe"
	elif lp == 9:
		return "Nothing Detected or System Unavailable"
	elif lp == 0:
		return "Velocity not detected"



def transform_callback3(data):
    global norm, conf_thr, conf, index, desired_keypoints
    conf = data.data
    index = desired_keypoints

        
    
def transform_callback2(data):
    global velocity
    velocity = data.data

def transform_callback1(data):
    global safety, frame_distance, frame_velocity, safety_value, conf_thr, index, conf, velocity, frame_conf,frame_dist_avg,frame_vel_avg,frame_dist_wtd,frame_vel_wtd
    global dmax,rt,vmax,amax,safety ,prev_d,d,v,time_prev,velo

    norm = 0
    ex = 0.00000000000000000000000000000000000001
    new_index = []
    for li in index:
    	if float(conf[li]) > conf_thr:
    	    new_index.append(li)
    
    print('third',new_index)
    
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    
    dd1 = []
    vv1 = []
    cc1 = []
    for i in new_index:
        dd1.append(data.data[i])
        vv1.append(velocity[i])
        cc1.append(conf[i])
    dd2 = []
    vv2 = []
    cc2 = []
    keep_index1 = []
    keep_index1 = [i for i,xd in enumerate(dd1) if math.isnan(xd) == False]
    for i in keep_index1:
        dd2.append(dd1[i])
        vv2.append(vv1[i])
        cc2.append(cc1[i])
    dd3 = []
    vv3 = []
    cc3 = []
    keep_index2 = []
    keep_index2 = [i for i,xv in enumerate(vv2) if math.isnan(xv) == False]
    
    for i in keep_index2:
        dd3.append(dd2[i])
        vv3.append(vv2[i])
        cc3.append(cc2[i])
        
    frame_distance = dd3[:]
    frame_velocity = vv3[:]
    frame_conf = cc3[:]
    
    norm = sum(frame_conf)
    print("norm :",norm)

    if len(frame_distance) != 0 :
        frame_dist_avg = sum(frame_distance)/len(frame_distance)
    if len(frame_velocity) != 0 :
        frame_vel_avg = sum(frame_velocity)/len(frame_velocity)
        
    frame_dist_wtd = 0
    frame_vel_wtd = 0
    if norm != 0:
        for i in range(len(frame_distance)):
            frame_dist_wtd = frame_dist_wtd + ((frame_conf[i]*frame_distance[i])/norm)
        
        for i in range(len(frame_velocity)):
            frame_vel_wtd = frame_vel_wtd + ((frame_conf[i]*frame_velocity[i])/norm)
    
    
    d = 0
    
    for i in range(len(frame_distance)):
        if norm != 0:
            d = d+(frame_conf[i]/norm)*((frame_distance[i]-dmin)/dmax)
        else:
            d = float('nan')
    if d>1 :
    	d = 1
    elif d<0 :
    	d = 0
    
    
    time_now = rospy.get_rostime().to_sec()


    v = 0
    for i in range(len(frame_velocity)):
        if norm == 0.0:
            v = float('nan')
        elif frame_distance[i] != float('nan') or frame_velocity[i] != float('nan'):
            allowedv = (frame_distance[i])/rt
            v = v + (frame_conf[i]/(norm))*((allowedv - frame_velocity[i])/(allowedv+ex))
        
    if v<0:
        v = 0
    elif v>1:
        v = 1
    
    if len(frame_distance) == 0 or len(frame_velocity) == 0:
        d = float('nan')
        v = float('nan')
    
    
    print("d =",d)
    print("v =",v)
    print("wtd_dist",frame_dist_wtd)
    
    if d == float('nan') or v == float('nan'):
        os_dist = float('nan')
        os_vel = float('nan')
        os_dist_wtd = float('nan')
        os_vel_wtd = float('nan')
        os_avg = float('nan')
    else: 
        os_dist = d
        os_vel = v
        os_dist_wtd = (0.75)*d+(0.25)*v
        os_vel_wtd = (0.25)*d+(0.75)*v
        os_avg = (0.5)*d+(0.5)*v
    

    if len(frame_distance) == 0 or os_avg == float('nan'):
    	lp = 9
    	safety = likert_scale(lp)
    elif os_avg < 0.1 :
    	lp = 1
    	safety = likert_scale(lp)
    elif os_avg >= 0.1 and os_avg < 0.28 :
    	lp = 2
    	safety = likert_scale(lp)
    elif os_avg >= 0.28 and os_avg < 0.45:
    	lp = 3
    	safety = likert_scale(lp)
    elif os_avg >= 0.45 and os_avg < 0.55:
    	lp = 4
    	safety = likert_scale(lp)
    elif os_avg > 0.55 and os_avg <= 0.73 :
    	lp = 5
    	safety = likert_scale(lp) 
    elif os_avg > 0.73 and os_avg <= 0.9 :
    	lp = 6
    	safety = likert_scale(lp)
    elif os_avg > 0.9 :
    	lp = 7
    	safety = likert_scale(lp)
    
    
    print('current distance :',frame_distance,'m distance factor :',d)
    print('confidence :',frame_conf)
    print('current velocity :',frame_velocity,'m/s velocity factor :',v)
    print('overall (distance+velocity) :',os_avg)
    print('avg vs wtd :',frame_dist_avg,frame_dist_wtd)
    print(safety)
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    time_prev = time_now
    frm_dist.data = frame_distance
    frm_vel.data = frame_velocity
    frm_conf.data = frame_conf
    pub.publish(String(safety))
    dist_arr.publish(frm_dist)
    vel_arr.publish(frm_vel)
    conf_arr.publish(frm_conf)
    dist_fact.publish(d)
    vel_fact.publish(v)
    pub31.publish(Float64(os_avg))
    pub32.publish(Float64(os_dist))
    pub33.publish(Float64(os_vel))
    pub34.publish(Float64(os_dist_wtd))
    pub35.publish(Float64(os_vel_wtd))
    pub11.publish(Float64(min(frame_distance, default=float('NaN'))))
    pub12.publish(Float64(max(frame_distance, default=float('NaN'))))
    pub13.publish(Float64(frame_dist_avg))
    pub14.publish(Float64(frame_dist_wtd))
    pub21.publish(Float64(min(frame_velocity, default=float('NaN'))))
    pub22.publish(Float64(max(frame_velocity, default=float('NaN'))))
    pub23.publish(Float64(frame_vel_avg))
    pub24.publish(Float64(frame_vel_wtd))
    
    


def dxl_control():

    global safety, frame_distance, frame_velocity, safety_value
    rospy.init_node('compatibility', anonymous=True)
    rospy.Subscriber('/confidence', Float64MultiArray, transform_callback3)
    rospy.Subscriber('/velocity', Float64MultiArray, transform_callback2)
    rospy.Subscriber('/distance', Float64MultiArray, transform_callback1)
    #rospy.Subscriber('/distance_btw', Float64MultiArray, transform_callback1)
    while not rospy.is_shutdown():
        rospy.spin()
        
    


if __name__ == '__main__':
    try:
        dxl_control()
    except rospy.ROSInterruptException:
        pass
