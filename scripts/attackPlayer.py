#!/usr/bin/env python

import rospy
import sys
import math
from std_msgs.msg import String
from enum import Enum
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from unitysim.msg import BoundingBox3d

# Class to get and store sensor data
class sensorData:
	def __init__(self, name, laserName):
		self.roboName = name
		self.hasData = False
		self.hasPlayerFinderData = False
		self.checkFire = True
		rospy.Subscriber("/"+self.roboName+"/"+laserName, LaserScan, self.callback)
		rospy.Subscriber("/"+self.roboName+"/closestPlayer", BoundingBox3d, self.playerFinderCallback)
		rospy.Subscriber("/"+name+"/canshoot", Int32, self.checkFireCallback)

	
	def checkFireCallback(self,data):
		self.checkFire = data == Int32(1)

	def canFire(self):
		return self.checkFire

	def callback(self, data):
		self.dataPack = data
		self.hasData = True

	def playerFinderCallback(self, data):
		self.playerFinderData = data
		self.hasPlayerFinderData = True

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

	def isPlayerFinderDataAvail(self):
		return self.hasPlayerFinderData

	def getPlayerRotation(self):
		return -self.playerFinderData.center.position.y

	def getPlayerSize(self):
		return self.playerFinderData.size.y

	
# class to calculate the next movement of the robot
class grabObjState:

	def __init__(self, name, laser):
		self.roboName = name
		self.laserName = laser
		self.sensorDataObj = sensorData(self.roboName, self.laserName)
		
	def calcVert(self, num, theta):
		rawVert = math.cos(theta) * num
		scaledVert = rawVert
		return scaledVert

	def calcTwist(self, myRanges, lenRanges):
		mSpeed = 1
		# get data
		angleIncrement = self.sensorDataObj.getAngleIncrement()
		angleMin = self.sensorDataObj.getAngleMin()
		rangeMax = self.sensorDataObj.getRangeMax()

		hRotation = self.sensorDataObj.getPlayerRotation()

		# set basic variables
		vertSum = 0
		counter = 0

		vertMovement = 0
		

		for x in myRanges:
			curRange = x
			if math.isinf(curRange):
				curRange = rangeMax
			theta = counter * angleIncrement + angleMin
			curRange /= rangeMax
			vertMovement += self.calcVert(curRange, theta) # Forward Vector


			counter += 1
		vertMovement /= len(myRanges)

		rTwist = Twist()
		size = self.sensorDataObj.getPlayerSize()
		rTwist.linear.x = self.clamp(8-size, -1, 1)
		rTwist.angular.z = self.clamp(hRotation, -1, 1)
		return rTwist



	def determineMovement(self):
		hasData = self.sensorDataObj.isDataAvail() and self.sensorDataObj.isPlayerFinderDataAvail()
		mTwist = Twist()
		if hasData == True:
			sensorRanges = self.sensorDataObj.getSensorData()
			mTwist = self.calcTwist(sensorRanges, len(sensorRanges))
		return mTwist

	def clamp(self, n, minn, maxn):
		return max(min(maxn, n), minn)

	def canShoot(self):
		hasData = self.sensorDataObj.isDataAvail() and self.sensorDataObj.isPlayerFinderDataAvail()
		if hasData:
			hRotation = self.sensorDataObj.getPlayerRotation()
			size = self.sensorDataObj.getPlayerSize()
			canFire = self.sensorDataObj.canFire()
			if canFire and size >= 7 and hRotation <= .2:
				return True
		else:
			return False




# class that handles the movement functions
class grabObjectController:

	def __init__(self, name, laser):
		self.roboName = name
		self.laserName = laser


		self.pub = rospy.Publisher("/"+str(name)+"/cmd_vel", Twist, queue_size = 10)
		self.FirePub = rospy.Publisher("/"+str(name)+"/cannon", String, queue_size = 10)
		

		rospy.Subscriber("/"+name+"/attackPlayer",Int32, self.attackPlayerCallback)
		self.rate = rospy.Rate(10)
		self.grabObjStateObj = grabObjState(name, laser)
		self.isActive = False




	def attackPlayerCallback(self, data):
		self.isActive = data == Int32(1)

	def run(self):
		while not rospy.is_shutdown():
			if self.isActive:
				movementTwist = self.grabObjStateObj.determineMovement()
				self.pub.publish(movementTwist)
				if self.grabObjStateObj.canShoot():
					self.FirePub.publish("Fire")
			self.rate.sleep()
			

# start of main
if __name__ == '__main__':
	rospy.init_node("Controller", anonymous=True)
	robotName = rospy.get_param("~roboname")
	laserName = rospy.get_param("~lasername")
	try:
		grabObjectControllerObj = grabObjectController(robotName, laserName)
		grabObjectControllerObj.run()
	except rospy.ROSInterruptException:
		print("ERROR: something happened")
