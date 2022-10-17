#!/usr/bin/env python
import cv2 as cv
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import *
mul = 5
shif = 250
canva = np.zeros((shif*2,shif*2,3),dtype = 'uint8')
route = canva
def draw_pos(inp):
	global canva,route,mul,shif
	pos = [inp.pose.pose.position.x-1,inp.pose.pose.position.y+1]
	pos[0]*=16
	pos[1]*=16
	pos = [-pos[0]*mul,pos[1]*mul]
	rospy.loginfo([int(pos[0]+shif),int(pos[1])+shif])
	cv.circle(canva,(int(pos[0])+shif,int(pos[1])+shif),2,(255,255,0),4)
	return

def draw_bar(inp):
	global route,canva,mul,shif
	for i in range(0,len(inp.data),2):
		pp = (int(inp.data[i]*mul)+shif,int(inp.data[i+1]*mul)+shif)
		rospy.loginfo(pp)
		if pp[0]<shif*2 and pp[1]<shif*2 and pp[0]>=0 and pp[1]>=0:
			cv.circle(route,pp,2,(255,0,255),4)
	cv.imshow('position map',route+canva)
	cv.waitKey(10)
if __name__ == "__main__":
	rospy.init_node('odom_drawer')
	rospy.Subscriber('/movbot_odom',Odometry,draw_pos,queue_size = 100)
	rospy.Subscriber('/barrier_coords',Float64MultiArray,draw_bar,queue_size = 100)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		rate.sleep()
