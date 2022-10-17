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
from std_msgs.msg import Float64MultiArray
flag = False


route = np.zeros((1000,1000,3),dtype = 'uint8')
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
disappearing_point = []
monitor_pos = []
pub = rospy.Publisher('barrier_coords',Float64MultiArray)
def sendcoord(inp):
	global pub
	# global ks
	# rospy.loginfo(ks)
	msg = Float64MultiArray()
	msg.data = inp
	pub.publish(msg)

def solve_eq(eq1,eq2):
	# rospy.loginfo(eq1)
	# rospy.loginfo(eq2)
	tmp = np.array([[eq1[0],eq1[1]],[eq2[0],eq2[1]]])
	tt = np.matmul(np.linalg.inv(tmp) , np.array([[eq1[2]],[eq2[2]]]))
	# rospy.loginfo(tt)
	return [tt[0][0],tt[1][0]]
def getpoint(I,p1,p2):
	global M,ks
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
def find_line(a,b):
	return [(b[1]-a[1])/(a[0]-b[0]),1,a[1]-a[0]*(a[1]-b[1])/(a[0]-b[0])]
def findpink(img):
	global x1,x2,y1,y2,z1,z2,m1,m2,now,pre,M,monitor_pos,ks,disappearing_point
	hsv = cv.cvtColor(img,cv.COLOR_BGR2HSV)
	dark_pink = np.array([165,255,255])
	light_pink = np.array([140,53,55])
	mask = cv.inRange(hsv,light_pink,dark_pink)
	pinks = cv.findContours(mask,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)
	pinks = imutils.grab_contours(pinks)
	centers = []
	for c in pinks:
		MM = cv.moments(c)
		# rospy.loginfo(M["m00"])
		cX = int(MM["m10"]/(MM["m00"]+0.00001))
		cY = int(MM["m01"]/(MM['m00']+0.00001))
		if([cX,cY] != [0,0]):
			centers.append([cX,cY])
		# cv.circle(mask,(cX,cY),2,(255,255,255),2)
	# centers.remove([0,0])
	# centers.remove([0,0])
	centers.sort()
	y2 = centers[0]
	centers.pop(0)
	y1 = centers[0]
	centers.pop(0)
	for i in range(len(centers)):
		centers[i] = [centers[i][1],centers[i][0]]
	centers.sort()
	# rospy.loginfo(len(centers))
	z2 = [centers[0][1],centers[0][0]]
	centers.pop(0)
	z1 = [centers[0][1],centers[0][0]]
	centers.pop(0)
	centers.sort(reverse = True)
	x2 = [centers[0][1],centers[0][0]]
	centers.pop(0)
	x1 = [centers[0][1],centers[0][0]]
	centers.pop(0)
	rospy.loginfo(y2)
	# rospy.loginfo(centers)
	# for i in centers:
		# rospy.loginfo(len(i))
	e1 = [(y2[1]-y1[1])/float(y2[0]-y1[0]),-1,-y2[1]+y2[0]*float(y2[1]-y1[1])/(y2[0]-y1[0])]
	# y1,x1 = x1,y1
	# y2,x2 = x2,y2
	e2 = [(x2[1]-x1[1])/float(x2[0]-x1[0]),-1,-x2[1]+x2[0]*float(x2[1]-x1[1])/(x2[0]-x1[0])]
	O = solve_eq(e1,e2)
	# cv.circle(mask,(int(O[0]),int(O[1])),2,(255,255,255),5)
	for i in range(len(O)):
		O[i] = int(O[i])
	# rospy.loginfo(O)
	# cv.line(mask,(y2[0],y2[1]),(O[0],O[1]),(255,255,255),4)
	# cv.circle(mask,(x1[0],x1[1]),2,(255,255,255),9)
	
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
	# rospy.loginfo(len(M[0,:]))
	y_to_xz = y1
	z_to_xy = z1
	tmp = getpoint(y_to_xz,0,2)
	line1 = [[0,1,0],[tmp[0],-1,tmp[1]]]
	tmp = getpoint(z_to_xy,0,1)
	line2 = [[0,0,1],[tmp[0],tmp[1],-1]]
	k = 1/(line1[1][0]*line2[1][1]/line2[1][0]-line1[1][1])
	monitor_pos = [1,0,0]+k*line1[1][1]
	# cv.imshow('testing',mask)

	x110 = (M[0,0]+M[0,3]+M[0,1])/(1-(1-k1)-(1-k2))
	y110 = (M[1,0]+M[1,3]+M[1,1])/(1-(1-k1)-(1-k2))
	# cv.circle(mask,(int(x110),int(y110)),2,(255,255,255),5)
	rospy.loginfo(str(x110)+','+str(y110))
	disappearing_point = solve_eq(find_line(y1,[x110,y110]),find_line(O,x1))
	rospy.loginfo('disappearing point real position:'+str(getpoint([disappearing_point[0],disappearing_point[1]+1],0,1)))
	oxline = find_line([x110,y110],y1)
	oyline = find_line(O,x1)
	cv.line(mask,(int(x110),int(y110)),(int(oxline[2]/oxline[0]),0),(255,255,0),2)
	cv.line(mask,(x1[0],x1[1]),(int(oyline[2]/oyline[0]),0),(255,255,0),2)
	# cv.line(mask,(O[0],O[1]),((O[0]-O[1]*(O[1]-x1[1])/(O[0]-x1[0])*O[1]),0),(255,255,0),3)
	rospy.loginfo(int(oxline[2]/oxline[0]))
	# cv.line(mask,(O[0],O[1]),(x1[0],x1[1]),(255,255,0),3)
	cv.circle(mask,(int(disappearing_point[0]),int(disappearing_point[1])),2,(255,255,255),4)
	# cv.circle(mask,(int(disappearing_point[0]),int(1)),2,(255,255,255),4)

	cv.imshow('testing',mask)
	cv.waitKey(2)
	rospy.loginfo('done')
	rospy.loginfo(ks)	
	return

def initial(inp):
	global pre,now
	bridge = CvBridge()
	try:
		img = bridge.imgmsg_to_cv2(inp,'bgr8')
		findpink(img)
		now = img
		pre = img
		# rospy.loginfo(type(now))
	except CvBridgeError:
		rospy.loginfo('error')
def cmpp(inp):
	global pre,now,ks,route,disappearing_point
	# return
	bridge = CvBridge()
	img = bridge.imgmsg_to_cv2(inp,'bgr8')
	# rospy.loginfo("A: "+str(type(now)))
	# rospy.loginfo(type(pre))
	pre = now
	now = img
	# return
	diff = cv.absdiff(pre,now)
	gray = cv.cvtColor(diff,cv.COLOR_BGR2GRAY)
	blur = cv.GaussianBlur(gray,(5,5),0)
	_,thresh = cv.threshold(blur,20,255,cv.THRESH_BINARY)
	dilated = cv.dilate(thresh,None, iterations = 3)
	_,contours,_ = cv.findContours(dilated,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
	out = pre
	coords = []
	for c in contours:
		(x,y,w,h) = cv.boundingRect(c)
		if cv.contourArea(c)<125:
			continue
		if y+h<disappearing_point[1]:
			continue
		cv.rectangle(pre,(x,y),(x+w,y+h),(255,0,0),3)
		coords.append(x+w//2)
		coords.append(y+h)
		kk = getpoint([coords[-2],coords[-1]],0,1)
		coords[-2] = (kk[0])
		coords[-1] = (kk[1])
		out = pre
	for i in range(0,len(coords),2):
		cv.circle(route,(int(coords[i]*10)+500,int(coords[i+1]*10)+500),2,(255,255,255),5)
	rospy.loginfo(coords)
	sendcoord(coords)
	# getpoint()
	# cv.imshow('tmp',route)
	cv.circle(out,(int(disappearing_point[0]),int(disappearing_point[1])),2,(0,255,0),4)
	cv.imshow('now',out)
	cv.waitKey(10)
	pre = now
	return
'''
  <param name="movingbot2" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/movingbot.urdf.xacro" />
  <node pkg="gazebo_ros" type="spawn_model" name="spawn_movingbot2" args="-urdf -model movingbot2 -x 9 -y -1.5 -z 1.0 -param movingbot2" />
'''
def fil(inp):
	global flag,pre,now
	if flag == False:
		initial(inp)
		flag = True
		return
	else:
		cmpp(inp)
	return



if __name__ == '__main__':
	flag = False
	rospy.init_node('monitor1',anonymous = True)
	rospy.Subscriber('/monitor/image',Image,fil,queue_size = 100)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		rate.sleep()