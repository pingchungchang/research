#!/usr/bin/env python
import rospy
from nav_msgs.msg import *
from std_msgs.msg import *
import numpy as np
from geometry_msgs.msg import *
import actionlib
from move_base_msgs.msg import *
import cv2 as cv
import matplotlib.pyplot as plt
import imutils
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError

flag = False

pre = []
now = []
x = []
y = []
z = []
def findpink(img):
	hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
	# dark_yellow = np.array([30,255,255])
	# light_yellow = np.array([25,55,55])
	# dark_white = np.array([255,0,255])
	# light_white = np.array([0,0,179])
	dark_pink = np.array([165,255,255])
	light_pink = np.array([140,53,55])
	mask = cv.inRange(hsv,light_pink,dark_pink)
	# output = cv.bitwise_and(img,img,mask = mask)
	pinks = cv.findContours(mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
	pinks = imutils.grab_contours(pinks)
	centers = []
	for c in pinks:
		M = cv.moments(c)
		# rospy.loginfo(M["m00"])
		cX = int(M["m10"]/(M["m00"]+0.00001))
		cY = int(M["m01"]/(M['m00']+0.00001))
		centers.append([cX,cY])
		cv.circle(mask,(cX,cY),2,(255,255,255),2)
	centers.remove([0,0])
	centers.remove([0,0])
	rospy.loginfo(centers)
	# for i in pinks[1]:
	# 	rospy.loginfo(len(i))
	# 	# cv.circle(mask,(i[0],i[1]),2,(255,255,255),2)
	cv.imshow('testing',mask)
	cv.waitKey(2)
	return

def initial(inp):
	bridge = CvBridge()
	try:
		img = bridge.imgmsg_to_cv2(inp,'bgr8')
		# return
		findpink(img)
		# cv.imshow('test',img)
		# cv.waitKey(2)
	except CvBridgeError:
		rospy.loginfo('error')

def fil(inp):
	global flag,pre,now
	pre = now
	now = inp
	if flag == False:
		initial(inp)
		# flag = True
		return
	else:
		cmp(inp)
	return
if __name__ == '__main__':
	flag = False
	rospy.init_node('monitor1',anonymous = True)
	rospy.Subscriber('/monitor/image',Image,fil,queue_size = 100)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		rate.sleep()