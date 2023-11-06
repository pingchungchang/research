#!/usr/bin/env python
import math
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
from std_msgs.msg import String

mul = 5
stamp = 0
shif = 250
height = 20/5.0
barfile = open('/home/pcc/barpos.txt','w')
posfile = open('/home/pcc/movpos.txt','w')
# posfile = open('/home/movpos.txt','a')
canva = np.zeros((shif*2,shif*2,3),dtype = 'uint8')
route = canva
zline = [1.5,-1]
flag = False
pub = 0;
tot = 0
lazy = {}
weigh = 0
sigma = 0


def draw_pos(inp):
	global canva,route,mul,shif,height
	pos = [inp.pose.pose.position.x-zline[0],inp.pose.pose.position.y-zline[1]]
	# posfile.write(str(pos[0])+','+str(pos[1])+','+str(int(stamp))+' ')
	pos[0] = pos[0]*height
	pos[1] = pos[1]*height
	pos = [-pos[0]*mul,pos[1]*mul]
	rospy.loginfo(str(stamp)+'real position;'+str([(-pos[0]),(pos[1])]))
	cv.circle(canva,(int(pos[0])+shif,int(pos[1])+shif),2,(255,255,0),4)
	cv.imshow(sys.argv[1]+'_position map',route+canva)
	cv.waitKey(10)
	return

def draw_bar(inp):
	global route,canva,mul,shif,height
	for i in range(0,len(inp),2):
                rospy.loginfo(str(stamp)+':'+str(inp))
                barfile.write(str(inp[i]/height)+','+str(inp[i+1]/height)+','+str(int(stamp))+' ')
		pp = (int(inp[i]*mul)+shif,int(inp[i+1]*mul)+shif)
		if pp[0]<shif*2 and pp[1]<shif*2 and pp[0]>=0 and pp[1]>=0:
			cv.circle(route,pp,2,(255,0,255),4)
	cv.imshow(sys.argv[2]+'_position map',route+canva)
	cv.waitKey(10)

def addcoord(inp):
	global lazy,wei,tot,sigma
	inp = str(inp)
	inp = inp[1:-1].split(',')
	name = inp[0]
	if name in lazy:
		return
	coord = [float(inp[1]),float(inp[2])]
	dist = (float(inp[3])**2+float(inp[4])**2+float(inp[5])**2)**0.5
	lazy.add(name)
	sigma += dist
	wei[0] += dist*coord[0]
	wei[1] += dist*coord[1]
	rospy.loginfo(str(tot)+str(lazy))
	if len(lazy) == tot:
		draw_bar([wei[0]/sigma,wei[1]/sigma])
		lazy = set()
	return

if __name__ == '__main__':
	global pub,tot,lazy,wei,sigma
	wei = [0,0]
	sigma = 0
	lazy = set()
	tot = int(sys.argv[1])
	rospy.Subscriber('/movbot_odom',Odometry,draw_pos,queue_size = 100)
	for i in range(0,tot):
		rospy.Subscriber(str(sys.argv[2+i])+'for_comb',String,addcoord,queue_size=100)
	rospy.init_node('merge_info',anonymous = True)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
		rate.sleep()


