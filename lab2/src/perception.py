#!/usr/bin/env python
import rospy 
import random 
import numpy as np 
import math 
import tf
import random 
import sys
from geometry_msgs.msg import Point 
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import LaserScan 
from visualization_msgs.msg import Marker 

class perception():
	def __init__(self):
		rospy.init_node('perception') 
		
		rospy.Subscriber('/base_scan', LaserScan, self.laser_input_callback) 

		self.point_publisher = rospy.Publisher('/perception', Marker, queue_size = 5) 
		rospy.spin()

	def pointConvert(self,data):
		
		
		dataPoints = data.ranges
		points = []
		
		i = 0
		while i < len(dataPoints):
			print(type(dataPoints[i]))
			if dataPoints[i] > 0 and dataPoints[i] < 3:
				t = data.angle_min + (i) * data.angle_increment
				points.append((dataPoints[i]* math.cos(t),dataPoints[i] * math.sin(t)))
			i += 1
		
		return points
	
	 


				
	def ransac(self, data):
		
		iterations = 0 
		bestFit = []
		
		k = 75
		t = .3
		d = 0
		dataLen = len(data)-1 
		while iterations < k:
			if dataLen > 1: 
				alsoInliers = []
				point1 = data[random.randint(0,dataLen)]
				point2 = data[random.randint(0,dataLen)]

				#model fitted to be maybeInliers 
				print (point2[1] - point1[1])
				print (point2[0] - point1[0])
				if (point2[0] - point1[0]) != 0:
					m = (point2[1] - point1[1])/(point2[0] - point1[0])	
					
					c = float(point2[1] - (m * point1[0]))

					for point in data:
						iPoint = (((point[0]) + m * (point[1]) - m*c)/(1+(m*m))) ,((m*(point[0]) + (m*m)*(point[1]) - (m*m)*c)/(1+(m*m)) + c)
						distance = math.sqrt((iPoint[0] - point[0])**2 + (iPoint[1] - point[1])**2)
						if t > float(distance):
							alsoInliers.append(point)
						 
						if len(alsoInliers) >= d:
							d = len(alsoInliers)
							betterModel = alsoInliers
							#betterModel = []
							betterModel.append(point1)
							betterModel.append(point2)

						
						if iterations == (k - 1):
							bestFit = betterModel

			iterations+=1

		if not bestFit: 
			bestFit.append((0,0))
			bestFit.append((0,0))
				
		
		
		return bestFit

			

			



		'''
		k = 50
		t = .3 
		d = 0

		it = 0 
		dataLen = len(data)
		betterModel =[]
		while it < k:
			#maybeInliers 			
			point1 = data(np.random.randint(0,dataLen))
			point2 = data(np.random.randint(0,dataLen))
			if point2 > point1:
				m = (point2[1] - point1[1])/(point2[0] - point1[0]) 
				c = point2[1] - m*point1[0]
			else: 
				m = (point1[1] - point2[1])/(point1[0] - point2[0])
				c = point1[1] - m*point2[0]
			
			alsoInliers = []
			# number of inliers 			
			countInliers = 0
			#every point not in maybeInliers

			for p in data:
				xData = p[0]
				yData = p[1]
				xIntercept = (xData + m * yData - m* c)/(1 + m**2) 
				yIntercept = (m*xData + (m**2)*yData - (m**2)*c)/(1+m**2)+c
				#distance from Intercept 
				line = math.sqrt((xIntercept - xData)**2 + (yIntercept - yData)**2)
				#if points fits maybe model with an smaller than threshhold				
				if t > line:
					#add points to alsoInliers
					alsoInliers.append(p) 
					countInliers = countInliers + 1
			    	#end for
			    	#if the number of elements in alsoInliers is > d then
				#This implies that we may have found a good model
				#now test how good it is.
				#used formula from (youtube.com/watch?v=BpOKB3OzQBQ&ab_channel=BobTrenwith)
				if float(countInliers)/float(len(data)) > d:
					d = float(countInliers)/float(len(data))				
					alsoInliers.append(point1)
					alsoInliers.append(point2)				
					bestModel = np.sort(alsoInliers)
				
		    	#increment iterations
			it+=1

		if len(bestModel) == 0: 
			bestModel.append(Point(1,1,0))
		return bestModel  
	
		'''
	

	def laser_input_callback(self,data): 

		#rate = rospy.Rate(100)
		#rospy.sleep(0.2)
		#while not rospy.is_shutdown():

		message = Marker()
		message.header.stamp = rospy.Time.now()
		message.header.frame_id = "/odom" 
		message.type = message.LINE_LIST
		message.action = message.ADD
		message.lifetime = rospy.Duration(10)
		message.scale.x = .1 
		#message.scale.y = .1
		message.color.a = 1.0 
		message.color.r = 1.0 

		#message.points.append(Point(1,1,0))
		points = self.pointConvert(data)
		dataPoints = self.ransac(points)

		if len(dataPoints) %2 != 0:
			dataPoints.pop(0)
		for point in dataPoints: 
			message.points.append(
				Point(point[0],point[1], 0)
			)

		self.point_publisher.publish(message) 
			#rate.sleep()


if __name__ == '__main__':
	try: 
		perception()
	except rospy.ROSInterruptException:
		pass 
	

