#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import random
import math 
import numpy as np

angle_inc = 0.00872664619237
A = 0
B = 0
C = 0

def callback(msg):
	
	a =[]
	M =[]
	C =[]
	po =[]
	u =[]
	v =[]
	num_iter = 100
	thresh_dist = 0.05
	thresh_num = 10

	for i in range(0,360):
		
		x = (msg.ranges[i]*math.cos((i*angle_inc)))
		y = (msg.ranges[i]*math.sin((i*angle_inc)))
		po.append([x,y])
		a.append([x,y])

	
	while(len(a)>thresh_num):
		m,c,a = rans(a,num_iter,thresh_dist,thresh_num)
		M.append(m)
		C.append(c)
		
	rospy.loginfo('M = '+ str(len(M)))
	#pub.publish(str(a))
	marker1 = Marker()
    	marker1.header.frame_id = "/base_link"
   	marker1.type = Marker.LINE_STRIP
    	marker1.header.stamp = rospy.Time.now()
    	marker1.ns = "scanlines"
    	marker1.action = Marker.ADD
    	marker1.pose.orientation.w = 1.0
    	marker1.id = 7
    	marker1.scale.x = 0.05
    	marker1.color.g = float(1.0)
    	marker1.color.a = 1.0
    	marker1.points = []
    	
	dub = rospy.Publisher("ransac_vis", Marker, queue_size=10)
	for i in range(0,360):
		p=Point()
		p.x = po[i][0]
		p.y = po[i][1]
		p.z = 0
		marker1.points.append(p)
	dub.publish(marker1)

	

	u=[]
	v=[]
	u = np.arange(-5,5,1)
	for i in range(0,len(u)):
		v.append(M[0]*u[i] + C[0])

	marker2 = Marker()
    	marker2.header.frame_id = "/base_link"
   	marker2.type = Marker.LINE_STRIP
    	marker2.header.stamp = rospy.Time.now()
    	marker2.ns = "line1"
    	marker2.action = Marker.ADD
    	marker2.pose.orientation.w = 1.0
    	marker2.id = 1
    	marker2.scale.x = 0.05
    	marker2.color.r = float(1.0)
    	marker2.color.a = 1.0
    	marker2.points = []
    	
	dub = rospy.Publisher("ransac_vis", Marker, queue_size=10)
	for i in range(0,len(u)):
		p=Point()
		p.x = u[i]
		p.y = v[i]
		p.z = 0
		marker2.points.append(p)
	dub.publish(marker2)
	
	u=[]
	v=[]
	u = np.arange(-5,5,1)
	for i in range(0,len(u)):
		v.append(M[1]*u[i] + C[1])

	marker3 = Marker()
    	marker3.header.frame_id = "/base_link"
   	marker3.type = Marker.LINE_STRIP
    	marker3.header.stamp = rospy.Time.now()
    	marker3.ns = "line2"
    	marker3.action = Marker.ADD
    	marker3.pose.orientation.w = 1.0
    	marker3.id = 2
    	marker3.scale.x = 0.05
    	marker3.color.b = float(1.0)
    	marker3.color.a = 1.0
    	marker3.points = []
    	
	dub = rospy.Publisher("ransac_vis", Marker, queue_size=10)
	for i in range(0,len(u)):
		p=Point()
		p.x = u[i]
		p.y = v[i]
		p.z = 0
		marker3.points.append(p)
	dub.publish(marker3)

	'''u=[]
	v=[]
	u = np.arange(-5,5,1)
	for i in range(0,len(u)):
		v.append(M[2]*u[i] + C[2])

	marker4 = Marker()
    	marker4.header.frame_id = "/base_link"
   	marker4.type = Marker.LINE_STRIP
    	marker4.header.stamp = rospy.Time.now()
    	marker4.ns = "line2"
    	marker4.action = Marker.ADD
    	marker4.pose.orientation.w = 1.0
    	marker4.id = 2
    	marker4.scale.x = 0.05
    	marker4.color.b = float(1.0)
    	marker4.color.a = 1.0
    	marker4.points = []
    	
	dub = rospy.Publisher("ransac_vis", Marker, queue_size=10)
	for i in range(0,len(u)):
		p=Point()
		p.x = u[i]
		p.y = v[i]
		p.z = 0
		marker4.points.append(p)
	dub.publish(marker4)'''
	
	   
def rans(points,num_iter,thresh_dist,thresh_num):

	mArray = []
	cArray = []
	outArray = []	
	for i in range(0,num_iter):
		p1,p2 = random.sample(points,2)
		
		count_in = 0 
		outliers = []
		m = (p2[1]-p1[1]) / (p2[0]-p1[0])
		c = ((p1[1]*p2[0]) - (p2[1]*p1[0])) / (p2[0]-p1[0])
		for j in range(len(points)):
			dist = abs((p2[0]-p1[0])*(p1[1]-points[j][1])-(p1[0]-points[j][0])*(p2[1]-p1[1]))/math.sqrt(pow((p2[0]-p1[0]),2)+pow((p2[1]-p1[1]),2))
			if ( dist < thresh_dist ):
				count_in += 1	
			else :
				outliers.append(points[j])
		if(count_in > thresh_num):
			thresh_num = count_in
			mArray.append(m)
			cArray.append(c)
			outArray.append(outliers)
	if(len(mArray) == 0 ):
		return [],[],[]
	else:
		return 	mArray.pop(),cArray.pop(),outArray.pop()
			

	
if __name__ == '__main__':
	rospy.init_node('perception')
	
	sub = rospy.Subscriber("base_scan",LaserScan,callback)
	
	rospy.spin()
