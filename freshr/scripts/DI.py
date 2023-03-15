#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64MultiArray
from std_msgs.msg import Float64 
import numpy as np
import math

global v_factor

v_factor = float('nan')
safety_value = float('nan')
dmax = 5 # Start detecting the distance once dmax is breached.
dmin = 0.5 # minimum separation distance that has to be maintained during the operation.
vmax = 2  # maximum speed of the robot is 2 m/s.
vmax = -2
amax = 0.7  # maximum acceleration of the robot is 0.7 m/s^2.
safety = "System Starting"


pub = rospy.Publisher('/danger_index', Float64, queue_size = 10)


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


def transform_callback2(data):
    global v_factor
    velocity = data.data
    kv = (1/(vmax - vmin))**2
    
    if math.isnan(velocity) == False:
    	if velocity < vmin :
    		v_factor = 0
        else:
        	v_factor = kv*((velocity-vmin)**2)    

def transform_callback1(data):
    global v_factor

    distance = data.data
    kd = (((dmin*dmax)/(dmin-dmax))**2)
    
    if math.isnan(disatnce) == False:
    	if distance > dmax :
    		d_factor = 0
        else:
        	d_factor = kd*(((1/distance)-(1/dmax))**2)
    
    
    
    if math.isnan(disatnce) == True :
        d = float('nan')
        v = float('nan')
    
    if d == float('nan') or v == float('nan'):
        os_avg = float('nan')
    else: 
        os_avg = 1-(d_factor*v_factor)
    

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
    
    pub.publish(Float64(os_avg))

    
    


def dxl_control():

    rospy.init_node('dangerindex', anonymous=True)
    rospy.Subscriber('/Framework/distance_wtd', Float64, transform_callback1)
    rospy.Subscriber('/Framework/velocity_wtd', Float64, transform_callback2)

    while not rospy.is_shutdown():
        rospy.spin()
        
    


if __name__ == '__main__':
    try:
        dxl_control()
    except rospy.ROSInterruptException:
        pass
