#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64MultiArray
from std_msgs.msg import Float64 
import numpy as np
import math



safety_value = float('nan')
threshold = 2.25 # distance beyond which everything is safe
dmin = 0.5 # minimum separation distance that has to be maintained during the operation.
safety = "System Starting"


pub = rospy.Publisher('/zonal_safety', Float64, queue_size = 10)


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


def transform_callback1(data):

    distance = data.data
    
    if math.isnan(disatnce) == False:
    	if distance < dmin :
    		d = 0
        elif distance > dmin and distance < threshold:
        	d = 0.5
        else :
        	d = 1
    
    
    
    if math.isnan(disatnce) == True :
        d = float('nan')
        v = float('nan')
    
    if d == float('nan') or v == float('nan'):
        os_avg = float('nan')
    else: 
        os_avg = d
    

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
    while not rospy.is_shutdown():
        rospy.spin()
        

if __name__ == '__main__':
    try:
        dxl_control()
    except rospy.ROSInterruptException:
        pass
