#!/usr/bin/env python
import cv2 as cv
import rospy
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import *
mul = 5
stamp = 0
shif = 250
height = 20/5.0
'''
barfile = open('/home/pcc/barpos.txt','w')
posfile = open('/home/pcc/movpos.txt','w')
'''
# posfile = open('/home/movpos.txt','a')
canva = np.zeros((shif*2,shif*2,3),dtype = 'uint8')
route = canva
zline = [1.5,-1]
def draw_pos(inp):
	global canva,route,mul,shif,height
	pos = [inp.pose.pose.position.x-zline[0],inp.pose.pose.position.y-zline[1]]
        #posfile.write(str(pos[0])+','+str(pos[1])+','+str(int(stamp))+' ')
	pos[0] = pos[0]*height
	pos[1] = pos[1]*height
	pos = [-pos[0]*mul,pos[1]*mul]
	rospy.loginfo(str(stamp)+'real position;'+str([(-pos[0]),(pos[1])]))
	cv.circle(canva,(int(pos[0])+shif,int(pos[1])+shif),2,(255,255,0),4)
	return

def draw_bar(inp):
	global route,canva,mul,shif,height
	for i in range(0,len(inp.data),2):
                rospy.loginfo(str(stamp)+':'+str(inp.data))
                #barfile.write(str(inp.data[i]/height)+','+str(inp.data[i+1]/height)+','+str(int(stamp))+' ')
		pp = (int(inp.data[i]*mul)+shif,int(inp.data[i+1]*mul)+shif)
		if pp[0]<shif*2 and pp[1]<shif*2 and pp[0]>=0 and pp[1]>=0:
			cv.circle(route,pp,2,(255,0,255),4)
	cv.imshow(sys.argv[1]+'_position map',route+canva)
	cv.waitKey(10)
if __name__ == "__main__":
	rospy.init_node('odom_drawer',anonymous = True)
	rospy.Subscriber('/movbot_odom',Odometry,draw_pos,queue_size = 100)
	rospy.Subscriber(str(sys.argv[1])+'barrier_coords',Float64MultiArray,draw_bar,queue_size = 100)
	rate = rospy.Rate(100)
	while not rospy.is_shutdown():
            stamp += 0.5
            # stamp += 0.5
            rate.sleep()
