#!/usr/bin/env python

import rospy 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point,Twist
from std_msgs.msg import String
# import random 
# import numpy as np 
import math 
from math import atan2

# import tf
# import random 
# import sys
from geometry_msgs.msg import Point 
# from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan 
from visualization_msgs.msg import Marker 

class Node:
	def __init__(self,po,a,g,v,pa):
		self.pos = po
		self.action = a
		self.value_g = g
		self.value = v
		self.parent = pa
'''
global tworld
tworld =[[0,0,0,0,0,0,0,0,0,0,0,0,1,x,1,0,0,0],
         [0,0,0,0,0,0,0,0,0,0,0,0,1,x,1,0,0,0],
         [0,0,0,0,0,0,0,0,0,0,0,0,x,0,0,0,0,0],
         [1,0,0,0,0,0,0,0,0,0,0,0,x,0,0,0,0,0],
         [0,1,0,0,0,0,0,0,0,0,0,0,x,0,0,0,0,0],
         [0,0,1,0,0,0,1,1,1,1,1,1,x,0,0,0,0,0],
         [0,0,1,0,0,0,1,1,1,1,1,1,x,0,0,0,0,0],
         [0,0,0,1,0,0,0,0,0,0,0,x,0,0,0,1,1,0],
         [0,0,0,0,1,0,0,0,0,0,x,0,0,0,0,1,1,1],
         [0,0,0,0,1,1,0,0,0,x,0,0,0,0,0,1,1,1],
         [0,0,x,x,0,1,0,0,0,x,0,0,0,0,0,1,1,1],
         [0,s,0,0,x,1,1,0,0,x,0,0,0,0,0,0,1,0],
         [0,0,0,0,0,x,1,1,1,x,0,0,0,0,0,0,0,0],
         [0,0,0,0,0,0,x,1,x,0,0,0,0,0,0,0,0,0],
         [0,0,0,0,0,0,0,x,0,0,0,0,0,0,0,0,0,0],
         [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
         [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
         [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
         [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
         [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]
'''
global world 
world = [[0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0],
       [0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0],
       [0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1],
       [0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0],
       [0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0],
       [0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1]]

'''
rospy.set_param('goalx',4)
rospy.set_param('goaly',9)


x = int(rospy.get_param('goalx'))
y = int(rospy.get_param('goaly'))

'''	

global world_start
world_start = (-8.0, -2.0)

global world_end
world_end = (rospy.get_param('goalx'),rospy.get_param('goaly'))

center = (9,9)
global start
start = (center[0]+2,center[1] - 8)
global goal
#goal = (center[0] - 9 , center[1] + 4)
goal = (center[0] + (-1 * int(rospy.get_param('goaly'))) , center[1] + int(rospy.get_param('goalx')))

#goal = (start[0] + 8,start[1] + 4)

#19.724
#print(math.sqrt((goal[0] - 10)**2 + (goal[1] - 9)**2) + math.sqrt((start[0] - 12)**2 + (start[1] - 9)**2))

global rob_x
global rob_y
global rob_t 


def distance(pos):
	last_pos = (rospy.get_param('goalx') , rospy.get_param('goaly'))
	int_last_pos = (int(rospy.get_param('goalx')) , int(rospy.get_param('goaly')))
	h = math.sqrt((last_pos[0] - pos.pos[0])**2 + (last_pos[1] - pos.pos[1])**2)
	g = pos.value_g + (last_pos[0] - float(int_last_pos[0])) + (last_pos[1] - float(int_last_pos[1]))

	f = h + g
	return f 

def step(pos,action):
	if action == 'U':
		return (pos[0] - 1, pos[1])
	if action == 'D':
		return (pos[0] + 1, pos[1])
	if action == 'R':
		return (pos[0], pos[1] + 1)
	if action == 'L':
		return (pos[0], pos[1] - 1)
	
	if action == 'UR':
		return (pos[0] - 1, pos[1] + 1)
	if action == 'RD':
		return (pos[0] + 1, pos[1] + 1)
	if action == 'DL':
		return (pos[0] + 1, pos[1] - 1)
	if action == 'LU':
		return (pos[0] - 1, pos[1] - 1)

def valid(pos, action):
	
	if action == 'UR':
		if pos[0] == 0 or pos[1] == 19:
			return False
		if world[pos[0] - 1][pos[1]] == 1 and world[pos[0]][pos[1] + 1] == 1:
			return False
		if world[pos[0] - 1][pos[1] + 1] == 1:
			return False
	if action == 'RD':
		if pos[0] == 19 or pos[1] == 19:
			return False
		if world[pos[0] + 1][pos[1]] == 1 and world[pos[0]][pos[1] + 1] == 1:
			return False
		if world[pos[0] + 1][pos[1] + 1] == 1:
			return False
	if action == 'DL':
		if pos[0] == 19 or pos[1] == 0:
			return False
		if world[pos[0] + 1][pos[1]] == 1 and world[pos[0]][pos[1] - 1] == 1:
			return False
		if world[pos[0] + 1][pos[1] - 1] == 1:
			return False
	if action == 'LU':
		if pos[0] == 0 or pos[1] == 0:
			return False
		if world[pos[0] - 1][pos[1]] == 1 and world[pos[0]][pos[1] - 1] == 1:
			return False
		if world[pos[0] - 1][pos[1] - 1] == 1:
			return False
	if action == 'U':
		if pos[0] == 0:
			return False
		if world[pos[0] - 1][pos[1]] == 1:
			return False
	if action == 'R':
		if pos[1] == 19:
			return False
		if world[pos[0]][pos[1] + 1] == 1:
			return False
	if action == 'D':
		if pos[0] == 19:
			return False
		if world[pos[0] + 1][pos[1]] == 1:
			return False
	if action == 'L':
		if pos[1] == 0:
			return False
		if world[pos[0]][pos[1] - 1] == 1:
			return False
	
	
	return True 

def lowest_value(nodes):
	low = nodes[0].value
	low_node = nodes[0]
	for x in nodes:
		if low >= x.value:
			low = x.value
			low_node = x
	return low_node

def path(node):
	final_path = []
	value_path = [] 
	action_path = []
	map_path = []
	current = node
	current_map = node
	current_pos = world_start
	
	while current.parent != None:
		action_path.insert(0,current.action)
		value_path.insert(0,current.value) 
		map_path.insert(0,current.pos)
		current = current.parent
		#print(current.action, current.pos)
#	print(action_path)


	for action in action_path:
		if action == 'U':
			x = current_pos[0]
			y = current_pos[1]
			current_pos = (x, y+1)
			final_path.append(current_pos)
		if action == 'R':
			x = current_pos[0]
			y = current_pos[1]
			current_pos = (x +1, y)
			final_path.append(current_pos)
		if action == 'D':
			x = current_pos[0]
			y = current_pos[1]
			current_pos = (x, y-1)
			final_path.append(current_pos)
		if action == 'L':
			x = current_pos[0]
			y = current_pos[1]
			current_pos = (x-1, y)
			final_path.append(current_pos)
		if action == 'UR':
			x = current_pos[0]
			y = current_pos[1]
			current_pos = (x +1, y+1)
			final_path.append(current_pos)
		if action == 'RD':
			x = current_pos[0]
			y = current_pos[1]
			current_pos = (x +1, y-1)
			final_path.append(current_pos)
		if action == 'DL':
			x = current_pos[0]
			y = current_pos[1]
			current_pos = (x -1, y-1)
			final_path.append(current_pos)
		if action == 'LU':
			x = current_pos[0]
			y = current_pos[1]
			current_pos = (x -1, y+1)
			final_path.append(current_pos)

	final_path[len(final_path)-1] = world_end
			
	return final_path,value_path,map_path
		
		

def planning():
	opened_nodes = []
	opened_pos = []
	closed = []
	actions = ['U','D','L','R','UR','RD','DL','LU']
	g = 0 
	#ask how to calculate the distance from current to end 
	h =  math.sqrt((goal[0] - start[0])**2 + (goal[1] - start[1])**2)
	
	new_node = Node((start[0],start[1]),None,g, g+h, None)
	opened_nodes.append(new_node)
	opened_pos.append((start[0],start[1]))
	
	
	while opened_nodes:
		current = lowest_value(opened_nodes)
		index = opened_nodes.index(current)
		index_pos = opened_pos.index(current.pos)
		opened_nodes.pop(index)
		closed.append(opened_pos.pop(index_pos))

		for action in actions: 
			if valid(current.pos,action):
				steped = step(current.pos,action)
				neighbor = (steped[0],steped[1])
				h = math.sqrt((goal[0] - neighbor[0])**2 + (goal[1] - neighbor[1])**2)
				# g = math.sqrt((start[0] - neighbor[0])**2 + (start[1] - neighbor[1])**2)
				g = current.value_g + math.sqrt((current.pos[0] - neighbor[0])**2 +  (current.pos[1] - neighbor[1])**2) 
				new_node = Node((neighbor[0],neighbor[1]), action,g, g+h, current)
				pos = (neighbor[0],neighbor[1])
				if pos not in closed and pos not in opened_pos:
					# print("pos :", pos)
					# print("closed :", closed)
					opened_pos.append(pos) 
					opened_nodes.append(new_node)

				if current.pos == goal:
					# print(current.pos)
					# print(current.action)
					return path(current)
				#calculate fcost 
				#append to open 


def rotate(): 
	rospy.init_node('robot_cleaner')
 	velocity_publisher = rospy.Publisher('robot_cleaner', anonymous = True) 
 	velocity_pubisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size = 10)
 	vel_msg = Twist()

 	PI = 3.1415926535897

 	angular_speed = 10 * 2 * PI/360
 	relative_angle = 1*2*PI/360 

 	vel_msg.linear.x = 0
 	vel_msg.linear.y = 0
 	vel_msg.linear.z = 0
 	vel_msg.angular.x = 0
 	vel_msg.angular.y = 0

 	vel_msg.angular.z = -abs(angular_speed)
	
 	t0 = rospy.Time.now().to_sec()
 	current_angle = 0 

 	while(current_angle < relative_angle):
 		velocity_publisher.publish(vel_msg)
 		t1 = rospy.Time.now().to_sec()
 		current_angle = angular_speed*(t1-t0)

 	vel_msg.angular.z = 0 
 	velocity_publisher.publish(vel_msg)
 	rospy.spin()

	
	
x = 0.0
y =0.0
angle_t = 0.0

def callback(msg):
	global x 
	global y 
	global angle_t 
	x = msg.pose.pose.position.x 
	y = msg.pose.pose.position.y

	rot = msg.pose.pose.orientation
	r, p, angle_t = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])

def move():
	value_path = []
	final_path = []
	map_path = []
	final_path,value_path,map_path = planning()
	rospy.init_node("robot")
	rospy.Subscriber("/odom", Odometry, callback)
	pub =rospy.Publisher("/cmd_vel",Twist,queue_size = 1)
	speed = Twist()
	r = rospy.Rate(10)

	numrows = len(world)
	numcols = len(world[0])

	for i in range(0,numrows):
		for j in range (0,numcols):
			if (i,j) in map_path: 
				print("x"),
			elif start == (i,j):
				print("S"),
			else:
				print(world[i][j]),
		print("\n")

	print(final_path)
	print(value_path)



	coor = final_path.pop()
	goal = Point()
	goal.x = coor[0]
	goal.y = coor[1]
	while not rospy.is_shutdown():
		to_x = goal.x - x
		to_y = goal.y - y
		angle_to_goal = atan2(to_y,to_x) 
		
		#print(angle_t)
		if abs(angle_to_goal - angle_t) > 0.1:
			speed.linear.x = 0.0 
			speed.angular.z = 0.1
		else:
			speed.linear.x = 0.5
			speed.angular.z = 0.0
		
		
		pub.publish(speed) 
		r.sleep()


if __name__ == '__main__':
	try: 
		move() 
	except rospy.ROSInterruptException: 
		pass 


