#!/usr/bin/env python
import cv2 as cv
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import *
canva = np.zeros((1000,1000,3),dtype = 'uint8')
route = canva
def draw_pos(inp):
	pos = [inp.pose.pose.position.x-1,inp.pose.pose.position.y+1]
	pos[0]*=8
	pos[1]*=8
	rospy.loginfo([pos[0]-1,pos[1]+1])
	pos = [-pos[0]*10,pos[1]*10]
	cv.circle(canva,(int(pos[0])+500,int(pos[1])+500),2,(255,255,0),4)
	cv.imshow('position map',route+canva)
	cv.waitKey(10)
	return

def draw_bar(inp):
	for i in range(0,len(inp.data),2):
		cv.circle(route,(int(inp.data[i]*10)+500,int(inp.data[i+1]*10)+500),2,(255,0,255),4)
if __name__ == "__main__":
	rospy.init_node('odom_drawer')
	rospy.Subscriber('/movbot_odom',Odometry,draw_pos,queue_size = 100)
	rospy.Subscriber('/barrier_coords',Float64MultiArray,draw_bar,queue_size = 100)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		rate.sleep()
