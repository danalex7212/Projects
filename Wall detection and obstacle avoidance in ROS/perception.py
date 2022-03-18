#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import random
import math 

angle_inc = 0.00872664619237


def callback(msg):
	a = []
    	b = []
	c = []
	
        #hello_str="false"
	for i in range(90,270):
    		if(msg.ranges[i]< 5.0 ):
			a.append(msg.ranges[i])
			b.append(msg.ranges[i]*math.cos(1.57079-(i*angle_inc)))
			c.append(msg.ranges[i]*math.sin(1.57079-(i*angle_inc)))
        
	b1= []
	b1 = b
	c1= []
	c1 = c

	num_iter = 10000
	thresh_dist = 0.005
	thresh_num = 50
	M = 0
	C = 0
	
	for i in range(num_iter):
		if(len(b1)>5):
			num_in = 0
			r1 = int(random.random()*len(b1))
			r2 = int(random.random()*len(c1))
			if r1 == r2:
				continue
			p1 = [b1[r1],c1[r1]]
			p2 = [b1[r2],c1[r2]]
			#rospy.loginfo('p1 = '+ str(p1))
			#rospy.loginfo('p2 = '+ str(p2))
			
			m = (p2[1]-p1[1])/(p2[0]-p1[0])
			#m = (c[r2]-c[r1])/(b[r2]-b[r1])	 
			d = p1[1]- m*p1[0]
			#d = c[r1] - m*b[r1]
			dist = 0
			for j in range(len(b)):
				dist = abs(((-m)*b1[j]+c1[j]-d)/math.sqrt(1+m*m))
				#rospy.loginfo(' p1x = '+str(p1[0])+' p1y = '+str(p1[1])+' p2x = '+str(p2[0])+' p2y = '+str(p2[1])+' m = '+str(m)+' c = '+str(d)+' b1 = '+str(b1[j])+' c1 = '+str(c1[j])+' dist = '+str(dist))
				if dist<thresh_dist:
					num_in += 1
			if(num_in > thresh_num):
				thresh_num = num_in
				M = m
				C = d
			rospy.loginfo('length of b  = '+str(len(b))+'i = '+str(i)+' num_in = '+str(num_in)+' thresh_num = '+str(thresh_num)+' M = '+str(M)+' C = '+str(C))
			b1.remove(b1[r1])
			c1.remove(c1[r1])
			b1.remove(b1[r2])
			c1.remove(c1[r2])
		
			
		
		
    #rate = rospy.Rate(1) # 10hz
    #while not rospy.is_shutdown():
    #hello_str = "hello world %s" % rospy.get_time()
    #rospy.loginfo(hello_str)
	pub = rospy.Publisher('chatter', String, queue_size=10)
	#str(len(a))+str(len(b))+str(len(c))
	#x = c.pop(int(random.random()*len(b)))
	pub.publish("M = "+str(M)+"  " + "C = "+str(C))
	
    	#rate.sleep() 
	   
	
if __name__ == '__main__':
	rospy.init_node('perception')
	#print("hello")
	#rospy.loginfo("Hello")
	#max_speed = rospy.get_param("~max_speed")
    	#max_steering = rospy.get_param("~max_steering_angle")
	#pub = rospy.Publisher("evader_drive",AckermannDriveStamped,queue_size=1)
	sub = rospy.Subscriber("base_scan",LaserScan,callback)
	
	
	rospy.spin()
	
