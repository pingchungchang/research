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
M = []
m1 = []
m2 = []
x1 = []
x2 = []
y1 = []
y2 = []
z1 = []
z2 = []
ks = []
monitor_pos = []
def solve_eq(eq1,eq2):
	# rospy.loginfo(eq1)
	# rospy.loginfo(eq2)
	tmp = np.array([[eq1[0],eq1[1]],[eq2[0],eq2[1]]])
	tt = np.matmul(np.linalg.inv(tmp) , np.array([[eq1[2]],[eq2[2]]]))
	# rospy.loginfo(tt)
	return [tt[0][0],tt[1][0]]
def getpoint(I,p1,p2):
	global M
	xx1 = M[0,p1]+(1-ks[p1])*I[0]
	yy1 = M[0,p2]+(1-ks[p2])*I[0]
	xx2 = M[1,p1]+(1-ks[p1])*I[1]
	yy2 = M[1,p2]+(1-ks[p2])*I[1]
	cc1 = I[0]-M[0,3]
	cc2 = I[1]-M[1,3]
	det = xx1*yy2-xx2*yy1
	aa = (yy2*cc1-yy1*cc2)/det
	bb = (xx1*cc2-xx2*cc1)/det
	return [aa,bb]
def findpink(img):
	pre = img
	global x1,x2,y1,y2,z1,z2,m1,m2,now,pre,M,monitor_pos,ks
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
		if([cX,cY] != [0,0]):
			centers.append([cX,cY])
		# cv.circle(mask,(cX,cY),2,(255,255,255),2)
	# centers.remove([0,0])
	# centers.remove([0,0])
	centers.sort()
	z2 = centers[0]
	centers.pop(0)
	z1 = centers[0]
	centers.pop(0)
	for i in range(len(centers)):
		centers[i] = [centers[i][1],centers[i][0]]
	centers.sort()
	# rospy.loginfo(len(centers))
	y2 = [centers[0][1],centers[0][0]]
	centers.pop(0)
	y1 = [centers[0][1],centers[0][0]]
	centers.pop(0)
	centers.sort(reverse = True)
	x2 = [centers[0][1],centers[0][0]]
	centers.pop(0)
	x1 = [centers[0][1],centers[0][0]]
	centers.pop(0)
	# cv.circle(mask,(y2[0],y2[1]),2,(255,255,255),5)

	# rospy.loginfo(centers)
	# for i in centers:
		# rospy.loginfo(len(i))
	e1 = [(y2[1]-y1[1])/float(y2[0]-y1[0]),-1,-y2[1]+y2[0]*float(y2[1]-y1[1])/(y2[0]-y1[0])]
	# y1,x1 = x1,y1
	# y2,x2 = x2,y2
	e2 = [(x2[1]-x1[1])/float(x2[0]-x1[0]),-1,-x2[1]+x2[0]*float(x2[1]-x1[1])/(x2[0]-x1[0])]
	O = solve_eq(e1,e2)
	for i in range(len(O)):
		O[i] = int(O[i])
	# rospy.loginfo(O)
	# cv.line(mask,(y2[0],y2[1]),(O[0],O[1]),(255,255,255),4)
	# cv.circle(mask,(O[0],O[1]),2,(255,255,255),5)
	
	m1 = [[O[0],x1[0],y1[0],z1[0]],[O[1],x1[1],y1[1],z1[1]],[1,1,1,1]]
	m1 = np.array(m1)
	m2 = [[x2[0],y2[0],z2[0]],[x2[1],y2[1],z2[1]],[1,1,1]]
	m2 = np.array(m2)
	A = np.linalg.inv(m2)
	A = A.dot(m1)

	k1 = A[2,0]/2/A[2,1]+A[1,0]/2/A[1,1]
	k2 = A[2,0]/2/A[2,2]+A[0,0]/2/A[0,2]
	k3 = A[0,0]/2/A[0,3]+A[1,0]/2/A[1,3]
	k1 /=2
	k2 /=2
	k3 /=2
	ks = [k1,k2,k3]
	M = m1.dot(np.array([[1,0,0,0],[0,k1,0,0],[0,0,k2,0],[0,0,0,k3]]))
	M = M.dot(np.array([[-1,-1,-1,1],[1,0,0,0],[0,1,0,0],[0,0,1,0]]))
	rospy.loginfo(len(M[0,:]))
	y_to_xz = y1
	z_to_xy = z1
	tmp = getpoint(y_to_xz,0,2)
	line1 = [[0,1,0],[tmp[0],-1,tmp[1]]]
	tmp = getpoint(z_to_xy,0,1)
	line2 = [[0,0,1],[tmp[0],tmp[1],-1]]
	k = 1/(line1[1][0]*line2[1][1]/line2[1][0]-line1[1][1])
	monitor_pos = [1,0,0]+k*line1[1][1]
	cv.imshow('testing',mask)
	cv.waitKey(2)
	rospy.loginfo('done')
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

def cmp(inp)



def fil(inp):
	global flag,pre,now
	pre = now
	now = inp
	if flag == False:
		initial(inp)
		flag = True
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