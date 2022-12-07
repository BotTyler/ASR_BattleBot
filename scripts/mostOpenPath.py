#!/usr/bin/env python

import rospy
import sys
import math
from std_msgs.msg import String
from enum import Enum
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32

# Class to get and store sensor data
class sensorData:
	def __init__(self, name, laserName):
		self.roboName = name
		self.hasData = False
		rospy.Subscriber("/"+self.roboName+"/"+laserName, LaserScan, self.callback)

	def callback(self, data):
		self.dataPack = data
		self.hasData = True

	def getSensorData(self):
		return self.dataPack.ranges

	def getAngleMin(self):
		return self.dataPack.angle_min

	def getAngleMax(self):
		return self.dataPack.angle_max

	def getAngleIncrement(self):
		return self.dataPack.angle_increment

	def getRangeMin(self):
		return self.dataPack.range_min

	def getRangeMax(self):
		return self.dataPack.range_max

	def isDataAvail(self):
		return self.hasData
	
# class to calculate the next movement of the robot
class mazeFinderState:

	def __init__(self, name, laser):
		self.roboName = name
		self.laserName = laser
		self.sensorDataObj = sensorData(self.roboName, self.laserName)

	def calcOpenForce(self, num, theta):
		rawHorz = math.sin(theta) * num
		scaledHorz = math.sin(.5 * math.pi * rawHorz)
		return scaledHorz
		
	def calcVert(self, num, theta):
		rawVert = math.cos(theta) * num
		scaledVert = rawVert
		return scaledVert

	def calcTwist(self, myRanges, lenRanges):
		mSpeed = 5
		# get data
		angleIncrement = self.sensorDataObj.getAngleIncrement()
		angleMin = self.sensorDataObj.getAngleMin()
		rangeMax = self.sensorDataObj.getRangeMax()

		# set basic variables
		horzSum = 0
		vertSum = 0
		counter = 0

		length = len(myRanges)
		forwardRange = length/3
		partialForwardRange = forwardRange/2
		middle = length/2
		upperVert = middle + partialForwardRange
		lowerVert = middle - partialForwardRange
		vertMovement = 0
		horzMoveToOpen = 0



		for x in myRanges:
			curRange = x
			if math.isinf(curRange):
				curRange = rangeMax
			theta = counter * angleIncrement + angleMin
			curRange /= rangeMax
			vertMovement += self.calcVert(curRange, theta) # Forward Vector
			horzMoveToOpen += self.calcOpenForce(curRange, theta) # Rotational Vector 1


			counter += 1
		vertMovement /= len(myRanges)
		horzMoveToOpen /= len(myRanges)

		
		rTwist = Twist()


		minVert = min(myRanges[lowerVert:upperVert+1])/rangeMax # / to get between 0 and 1


		if minVert <= .04:
			vertMovement = 0
			#horzMoveToOpen = self.clamp(horzMoveToOpen*1000, -1,1)
			

		rTwist.linear.x = vertMovement*mSpeed

		rTwist.angular.z = horzMoveToOpen * mSpeed
		return rTwist



	def determineMovement(self):
		hasData = self.sensorDataObj.isDataAvail()
		mTwist = Twist()
		if hasData == True:
			sensorRanges = self.sensorDataObj.getSensorData()
			mTwist = self.calcTwist(sensorRanges, len(sensorRanges))
		return mTwist

	def clamp(self, n, minn, maxn):
		return max(min(maxn, n), minn)




# class that handles the movement functions
class mazeFinderController:

	def __init__(self, name, laser):
		self.roboName = name
		self.laserName = laser

		
		self.pub = rospy.Publisher("/"+str(name)+"/cmd_vel", Twist, queue_size = 10)
		rospy.Subscriber("/"+name+"/mostOpenPath",Int32, self.mostOpenPathCallback)
		self.rate = rospy.Rate(10)
		self.mazeFinderStateObj = mazeFinderState(name, laser)
		self.isActive = False

	def mostOpenPathCallback(self, data):
		self.isActive = data == Int32(1)

	def run(self):
		while not rospy.is_shutdown():
			if self.isActive:
				movementTwist = self.mazeFinderStateObj.determineMovement()
				self.pub.publish(movementTwist)	
			self.rate.sleep()
			

# start of main
if __name__ == '__main__':
	rospy.init_node("Controller", anonymous=True)
	robotName = rospy.get_param("~roboname")
	laserName = rospy.get_param("~lasername")
	try:
		mazeFinderObj = mazeFinderController(robotName, laserName)
		mazeFinderObj.run()
	except rospy.ROSInterruptException:
		print("ERROR: something happened")
