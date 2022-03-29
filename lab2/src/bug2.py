#! /usr/bin/env python
import rospy 
import random 
import numpy as np 
import math 
import tf
import random 
from geometry_msgs.msg import Point 
from nav_msgs.msg import Odometry 
from sensor_msg.msg import LaserScan 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Twist 



global position 
position = [-8.0,-2.0]
global goal 
goal = [4.5,9.0] 
global isWall 
isWall = False 
global isLine 
isLine = False
	
class bug2
	def _init_(self):
		#rospy.init_node('bug2')
		rospy.init_node("speed_controller")

		#rospy.Subscriber('/base_scan', LaserScan, self.laser_input_callback)
		#rospy.Subscriber('/base_scan', LaserScan, self.laser_input_callback)
		rospy.Subscriber('/base_pose_ground_truth', Odemetry, self.newOdom)
		rospy.Publisher('cmd_vel', Twist, queue_size = 1)
		speed = Twist()
		
		r = rospy.Rate(4)

		self.point_publisher = rospy.Publisher('/perception', Marker, queue_size =1)
		rospy.spin()

	def newOdom(msg) 
		global x
		global y 
		global theta 
		
		x = msg.pose.pose.position.x 
		y = msg.pose.pose.position.y
		
		rot_q = mes.pose.pose.orientation 
		(roll, pitch, theta) = euler_from_quaternion(rot_q.x,rot_q.y,rot_q.z,rot_q.w)

	def line():
		initial = Point(-8.0,-2.0)
		goal = Point(4.5,9.0)

		m = (goal[1] - initial[1])/(goal[0] - initial[0]) 
		c = goal[1] - m*initial[0]

		xIntercept = (xData + m * yData - m* c)/(1 + m**2) 
		yIntercept = (m*xData + (m**2)*yData - (m**2)*c)/(1+m**2)+c
		dis = math.sqrt((xIntercept - xData)**2 + (yIntercept - yData)**2)
		
		return dis  

	def move():
		m = (goal[1] - initial[1])/(goal[0] - initial[0]) 
		c = goal[1] - m*initial[0]

		xIntercept = (xData + m * yData - m* c)/(1 + m**2) 
		yIntercept = (m*xData + (m**2)*yData - (m**2)*c)/(1+m**2)+c
		line = math.sqrt((xIntercept - xData)**2 + (yIntercept - yData)**2)
		angle = math.atan((points[1,1] - ps 
		rate = rospy.Rate(10) 
		t = 0.3 
		twist = Twist()
		
		while not rospy.is_shutdown():	
			inc_x = goal.x - x 
			inc_x = goal.y - y
			angle_to_goal = atan2(inc_y,inc_x)
			if abs(angle_to_goal - theta) > 0.1:
				twist.linear.x = 0.0 
				twist.angular.z = 0.3
			else:
				twist.linear.x = 0.5 
				twist.angular.z = 0.0
			
			if [x,y] == goal: 
				twist.linear.x = 0

			pub.publish(twist)
			rate.sleep()


	def laser_input_callback(self, data):
		message = Marker()
		message.header.stamp = rospy.Time.now()
		message.header.fram_id = "/odom"
		message.type = Marker.Line_List
		#message.type = message.POINT
		message.action = message.ADD
		message.lifetime = rospy.Duration(10)
		message.scale.x = .1 
		message.scale.y = .1 
		message.color.a = 1.0 
		message.color.r = 1.0
		p1 = Point()
		p2 = Point()
		
		message.points.append(ransac())
		
		

		self.point_publisher.publish(message)

#if _name_ == '_main_':
	
