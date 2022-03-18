#!/usr/bin/env python

#from sys import _OptExcInfo
from distutils import dist
from os import remove
from numpy.core.numeric import Inf
from numpy.distutils.npy_pkg_config import _read_config_imp
from numpy.f2py.crackfortran import true_intent_list
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import random
import math 

from nav_msgs.msg import Odometry
from tf import transformations

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

import numpy as np

angle_inc = 0.00872664619237
A = 0
B = 0
C = 0

position_ = Point()
yaw_ = 0
state_ = 0
bot_state_ = 0
des_point_ = Point()

des_point_.z = 0
des_yaw = 0
yaw_prec_ = 0.15#math.pi/90#90
dist_prec_ = 0.5
pub = None
count = 2

map = [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,1,1,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,
       0,0,1,1,0,0,1,1,1,1,1,1,1,0,0,0,0,0,
       0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1,
       0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0,
       0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
       0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0,
       0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]

obst_map = np.zeros(shape=(20,18))

class node():
	def __init__(self,par=None,pos=None):
		self.gcost = 0
		self.hcost = 0
		self.fcost = 0
		self.parent = par
		self.posit = pos
	
	def __eq__(self,an):
		return self.posit == an.posit

def astar(map,start,end):

	rows = len(map)
	cols = len(map[rows-1])
	#rospy.loginfo(rows)
	#rospy.loginfo(cols)
	start_node = node(None,start)


	start_node.gcost = 0
	start_node.hcost = 0
	start_node.fcost = 0
	

	end_node = node(None,end)

	end_node.gcost = 0
	end_node.hcost = 0
	end_node.fcost = 0

	#start_node.hcost = get_distance(start_node,end_node)
	
	open_nodes =[]
	visited_nodes =[]
	open_nodes.append(start_node)

	while(len(open_nodes)>0):
		
		#minf = Inf
		rospy.loginfo(len(open_nodes))
		id = 0
		current = open_nodes[0]
		for i in range(len(open_nodes)):
			if open_nodes[i].fcost<current.fcost :

				id = i
				current = open_nodes[i]
				#rospy.loginfo(current.hcost)
				minf = open_nodes[i].fcost
				
			""" if open_nodes[i].fcost == current.fcost :
				if open_nodes[i].hcost < minh:
					id = i
					current = open_nodes[i]
					minf = open_nodes[i].fcost
					minh = open_nodes[i].hcost """
		rospy.loginfo(current.posit)
		open_nodes.pop(id)
		if current in visited_nodes:
			continue
		
		visited_nodes.append(current)
		
		#rospy.loginfo(current.posit)
		if current == end_node:
			rospy.loginfo("FINISH")
			path =[]
			curr = current
			while curr != None :
				path.append(curr.posit)
				curr = curr.parent
			return path
		
		children =[]
		children = get_neighbors(current,rows,cols)
		

		for child in children : 
			var = False
			var1 = False
			#for c in visited_nodes:
			#	if child == c:
					
			#		var = True
			#		break
			#if var == True:
			if child in visited_nodes:
				continue
			child.gcost = current.gcost+get_distance(current,child)
			child.hcost = get_distance(child,end_node)
			#rospy.loginfo(child.gcost)
			child.fcost = child.gcost + child.hcost
			remove_l = []
			for o in open_nodes:

				if child == o and child.gcost>o.gcost:
					
					#rospy.loginfo("same")
					var1= True
					break
				elif child == o and child.gcost<o.gcost:
					remove_l.append(o)
			for j in remove_l:
				open_nodes.remove(j)
				#rospy.loginfo("removed")
			
			if var1 == True:
				continue
			
			open_nodes.append(child)
		
		



def get_distance(start,end):
	return math.sqrt((start.posit[0]-end.posit[0])**2 + (start.posit[1]-end.posit[1])**2)

def get_neighbors(cnode,rows,cols):
	l =[]
	neighbors =[]
	if cnode.posit[0] == 0 and cnode.posit[1]==0:
		l = [(0,1),(1,1),(1,0)]
	elif  cnode.posit[0]== (rows-1) and cnode.posit[1] == 0 :
		l = [(rows-2,0),(rows-2,1),(rows-1,1)]
	elif  cnode.posit[0] == (rows-1) and cnode.posit[1] == cols-1 :
		l = [(rows-1,cols-2),(rows-1,cols-2),(rows-2,cols-1)]
	elif  cnode.posit[0] == 0 and cnode.posit[1] == cols -1 :
		l = [(0,cols-2),(1,cols-2),(1,cols-1)]
	elif cnode.posit[0] != 0 and cnode.posit[1] == 0 :
		l = [(cnode.posit[0]-1,0),(cnode.posit[0]-1,1),(cnode.posit[0],1),(cnode.posit[0]+1,1),(cnode.posit[0]+1,0)]
	elif cnode.posit[0] != 0 and cnode.posit[1] == cols-1 :
		l = [(cnode.posit[0]-1,cols-1),(cnode.posit[0]-1,cols-2),(cnode.posit[0],cols-2),(cnode.posit[0]+1,cols-2),(cnode.posit[0]+1,cols-1)]
	elif cnode.posit[0] == 0 and cnode.posit[1] != 0 :
		l = [(0,cnode.posit[1]-1),(1,cnode.posit[1]-1),(1,cnode.posit[1]),(1,cnode.posit[1]+1),(0,cnode.posit[1]+1)]
	elif cnode.posit[0] == rows-1 and cnode.posit[1] != 0 :
		l = [(rows-1,cnode.posit[1]-1),(rows-2,cnode.posit[1]-1),(rows-2,cnode.posit[1]),(rows-2,cnode.posit[1]+1),(rows-1,cnode.posit[1]+1)]
	else:
		l = [(cnode.posit[0]-1,cnode.posit[1]-1),(cnode.posit[0]-1,cnode.posit[1]),(cnode.posit[0]-1,cnode.posit[1]+1),(cnode.posit[0],cnode.posit[1]+1),(cnode.posit[0]+1,cnode.posit[1]+1),(cnode.posit[0]+1,cnode.posit[1]),(cnode.posit[0]+1,cnode.posit[1]-1),(cnode.posit[0],cnode.posit[1]-1)]
	#rospy.loginfo(l)
	for pos in l : 
		
		if obst_map[pos[0]][pos[1]] == 0 :
			
				neighbors.append(node(cnode,pos))
	
	return neighbors

def point_to_point():
	global count,state_
	if state_ == 0 : 
		yaw(des_point_)
	elif state_ ==1 :
		straight(des_point_)	
	elif state_ ==2 :
		#done()
		count += 1
		state_ = 0
	else :
		rospy.loginfo("Unknown State")

def call_odom(msg) : 
	global position_
	global yaw_
	
	position_ = msg.pose.pose.position
	
	quat = (
		 msg.pose.pose.orientation.x,
		 msg.pose.pose.orientation.y,
		 msg.pose.pose.orientation.z,
		 msg.pose.pose.orientation.w		
		)
	euler = transformations.euler_from_quaternion(quat)

	yaw_ = euler[2]

def yaw(point):
	global position_
	global yaw_
	global pub
	global des_yaw
	#rospy.loginfo("came to yaw")
	
	yaw = Twist()
	
	#des_yaw = math.atan((point.y - position_.y)/(point.x - position_.x))
	
	change_yaw = des_yaw - yaw_
	#rospy.loginfo("change_yaw : "+str(change_yaw)+" des_yaw : "+str(des_yaw)+" x "+str(position_.x)+" y "+str(position_.y))
	
	if (math.fabs(change_yaw) > yaw_prec_):
		if (change_yaw > 0):
			yaw.angular.z = 0.5*math.fabs(change_yaw)#0.3 
		else :
			yaw.angular.z = -0.5*math.fabs(change_yaw)#-0.3
		pub.publish(yaw)
	else:
		change_state(1)

def straight(point):
	global position_,yaw_
	global des_yaw
	
	move = Twist()
	
	
	dist = math.sqrt(pow((point.y-position_.y),2)+pow((point.x-position_.x),2))
	des_yaw = math.atan(point.y - position_.y)/(point.x - position_.x)
	change_yaw = des_yaw - yaw_
	#rospy.loginfo("came to straight"+str(dist)+" des_yaw "+str(des_yaw)+" x "+str(position_.x)+" y "+str(position_.y))
	if (dist > dist_prec_):
		#rospy.loginfo("move forward")
		move.linear.x = 0.8*dist
		pub.publish(move)
	else:
		#rospy.loginfo("stop")
		change_state(2)

	if (math.fabs(change_yaw) > yaw_prec_):
		#rospy.loginfo("Change yaw")
		change_state(0)
	

def done():
	stop = Twist()
	stop.linear.x = 0.0
	stop.angular.x = 0.0
	pub.publish(stop)

def change_state(state):
	global state_
	state_=state
	#rospy.loginfo("Changed state to "+str(state))

if __name__ == '__main__':
	rospy.init_node('astar')
	global des_point_,state_,pub
	sub_odom = rospy.Subscriber("/odom",Odometry,call_odom)
	pub = rospy.Publisher("/cmd_vel",Twist,queue_size=1)
	endx = -1*(rospy.get_param('y')-10) #1#7#1#rospy.get_param('x') 
	endy = math.floor(rospy.get_param('x') + 9)#13#6#13.5#rospy.get_param('y') - 10
	
	startx = 12#8#0#8#-8 + 9
	starty = 1#1#0#1#-2 - 10

	rate = rospy.Rate(30)

	for i in range(len(map)):
		if map[i]==1:
			obst_map[int(i/18)][i%18] = 1
	
	#rospy.loginfo(startx,starty,endx,endy)
	result = astar(obst_map,(startx,starty),(endx,endy))
	path = []#[(-8, -2), (-7, -2), (-6, -2), (-5, -2), (-4, -3), (-3, -4), (-2, -5), (-1, -5), (0, -4), (0, -3), (0, -2), (0, -1), (0, 0), (0, 1), (1, 2), (2, 3), (3, 4), (3, 5), (3, 6), (3, 7), (3, 8), (4, 9)]#[(-8, -2), (-7, -2), (-6, -2), (-5, -2), (-4, -2), (-3, -3), (-2, -4), (-1, -3), (0, -2), (0, -1), (0, 0), (0, 1), (1, 2), (2, 3), (3, 4), (3, 5), (3, 6), (3, 7), (3, 8), (4, 9)] #[]
	result.reverse()
	for i in result : 
    			a = i[1]-9
    			b = -i[0]+10
    			path.append((a,b))

	rospy.loginfo(path)
	
	
	
	while not rospy.is_shutdown():
		if count == len(path)-1:
			state_ = 3
		des_point_.x = path[count][0]#4.5
		des_point_.y = path[count][1]#9.0
		#rospy.loginfo(str(state_))
		point_to_point()
		
		rate.sleep()
	

	
