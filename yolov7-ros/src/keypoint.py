#!/usr/bin/python
# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import torch
import cv2
import numpy as np
import time
import rospy
import math
from std_msgs.msg import String, Float64,Int32MultiArray, Float64MultiArray
from torchvision import transforms
from utils.datasets import letterbox
from utils.general import non_max_suppression_kpt
from utils.plots import output_to_keypoint, plot_skeleton_kpts
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#rosrun topic_tools throttle messages gazebo/model_states 1.0
#assumption "keypoints": [ "nose", "right_shoulder", "right_elbow", "right_wrist", "left_shoulder", "left_elbow", "left_wrist", "right_hip", "right_knee", "right_ankle", "left_hip", "left_knee", "left_ankle", "right_eye", "left_eye", "right_ear", "left_ear"]

#actual "keypoints": [ "nose", "left_eye", "right_eye", "left_ear", "right_ear", "left_shoulder", "right_shoulder", "left_elbow", "right_elbow", "left_wrist", "right_wrist", "left_hip", "right_hip", "left_knee", "right_knee", "left_ankle", "right_ankle"]


global depth_image
my_msg = Float64MultiArray()
mymsg = Float64MultiArray()
bbox = Float64MultiArray()
global color_box
color_box = [255,0,0]
safety = "Unsafe"
depth_image = 0

def depth_process(img):
	global depth_image,depth_img
	bridge=CvBridge()
	depth_image = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
	#cv2.imshow('image1', depth_image)
	#cv2.waitKey(1)

def get_color_box(data):
	global color_box,safety
	safety = data.data
	if data.data == "Safe":
		color_box = [0,255,0]
	elif data.data == "Mostly Safe":
		color_box = [0,191,0]
	elif data.data == "Slightly Safe":
		color_box = [170,255,170]
	elif data.data == "Neither Safe nor Unsafe (Neutral)":
		color_box = [0,255,255]
	elif data.data == "Slightly Unsafe":
		color_box = [0,178,255]
	elif data.data == "Mostly Unsafe":
		color_box = [0,89,255]
	elif data.data == "Unsafe":
		color_box = [0,0,255]
	else:
		color_box = [255,0,0]

def color_image(image_msg):
	global color_intrin
	bridge = CvBridge()
	cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")
	frame_width = image_msg.width
	frame_height = image_msg.height
	keypoint_est(cv_image, frame_width, frame_height)
	cv2.waitKey(1)

    
def keypoint_est(data, frame_width, frame_height):
	global nimg, color_box,safety
	device = torch.device(('cuda:0' if torch.cuda.is_available() else 'cpu'))
	print(device)
    

	weigths = torch.load('/home/pandey/yolov7/yolov7-w6-pose.pt')#, map_location='cuda:0')
	model = weigths['model']
	model = model.half().to(device)
	_ = model.eval()


	orig_image = data
	image = cv2.cvtColor(orig_image, cv2.COLOR_BGR2RGB)
	image = letterbox(image, frame_width, stride=64, auto=True)[0]
	image_ = image.copy()
	image = transforms.ToTensor()(image)
	image = torch.tensor(np.array([image.numpy()]))
	image = image.to(device)
	image = image.half()

	with torch.no_grad():
		(output, _) = model(image)

	output = non_max_suppression_kpt(
		output,
		0.25,
		0.65,
		nc=model.yaml['nc'],
		nkpt=model.yaml['nkpt'],
		kpt_label=True,
		)
	output = output_to_keypoint(output)
	xmin = 0.0
	xmax = 0.0
	ymin = 0.0
	ymax = 0.0
	human_key = output
	#robot_key = output[1]
	publishing_data = human_key.flatten()
	#robot_keyp_data = robot_key.flatten()
	#print(output.shape[0])
	print(publishing_data[:6])
	#print(robot_keyp_data[:6])
	nimg = image[0].permute(1, 2, 0) * 0xFF
	nimg = nimg.cpu().numpy().astype(np.uint8)
	nimg = cv2.cvtColor(nimg, cv2.COLOR_RGB2BGR)
	copy_img = depth_image
	
	for idx in range(output.shape[0]):
		plot_skeleton_kpts(nimg, output[idx, 7:].T, 3)
		# Comment/Uncomment the following lines to show bounding boxes around persons.

		(xmin, ymin) = (output[idx, 2] - output[idx, 4] / 2,
			output[idx, 3] - output[idx, 5] / 2)
		(xmax, ymax) = (output[idx, 2] + output[idx, 4] / 2,
			output[idx, 3] + output[idx, 5] / 2)
		cv2.rectangle(
		nimg,
		(int(xmin), int(ymin)),
		(int(xmax), int(ymax)),
		color=(color_box[0],color_box[1],color_box[2]),
		thickness=1,
		lineType=cv2.LINE_AA,
		)
		
		#offset = 17
		#cv2.circle(copy_img, (int(publishing_data[7]),int(publishing_data[8])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[10]),int(publishing_data[11])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[13]),int(publishing_data[14])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[16]),int(publishing_data[17])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[19]),int(publishing_data[20])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[22]),int(publishing_data[23])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[25]),int(publishing_data[26])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[28]),int(publishing_data[29])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[31]),int(publishing_data[32])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[34]),int(publishing_data[35])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[37]),int(publishing_data[38])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[40]),int(publishing_data[41])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[43]),int(publishing_data[44])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[46]),int(publishing_data[47])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[49]),int(publishing_data[50])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[52]),int(publishing_data[53])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.circle(copy_img, (int(publishing_data[55]),int(publishing_data[56])-offset), radius=1, color=(0 ,0 ,0), thickness=-1)
		#cv2.imshow('depth_image', copy_img)

	#cv2.circle(depth_image, (int(publishing_data[34]),int(publishing_data[35])), radius=1, color=(255, 255, 255), thickness=-1)
	cv2.imshow('color_image', nimg)
	#cv2.imshow('depth_image', depth_image)
	my_msg.data = publishing_data[7:]
	#mymsg.data = robot_keyp_data[7:]
	bbox.data = [xmin,ymin,xmax,ymax]
	pub1.publish(my_msg)
	pub2.publish(bbox)
	#pub3.publish(mymsg)



if __name__ == '__main__':
	
	rospy.init_node('yolo_classify', anonymous=True)
	pub1 = rospy.Publisher('/human_skeleton_keypoints', Float64MultiArray, queue_size = 1)
	pub3 = rospy.Publisher('/robot_keypoints', Float64MultiArray, queue_size = 1)
	pub2 = rospy.Publisher('/bounding_box', Float64MultiArray, queue_size = 1)
	#rospy.Subscriber('/realsense/color/image_raw', Image, color_image, queue_size = 1, buff_size = 16777216)
	rospy.Subscriber('/realsense/depth/image_rect_raw',Image,depth_process)
	rospy.Subscriber('/realsense/color/image_raw', Image, color_image, queue_size = 1, buff_size = 16777216)
	#rospy.Subscriber('/camera/aligned_depth_to_color/image_raw',Image,depth_process)
	rospy.Subscriber('/Framework/safety', String, get_color_box)
	while not rospy.is_shutdown():
		rospy.spin()

