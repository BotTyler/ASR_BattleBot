#!/usr/bin/env python

import rospy
import sys
import math
from std_msgs.msg import String
from enum import Enum
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import Int32

# Robot will move based on states
class STATES(Enum):
	FORWARD = 1
	RIGHT = 2
	LEFT = 3
	WANDER = 4
	CORNER = 5

#Class to get and hold the sensor data
class sensorData:
	def __init__(self, name, laserName):
		self.roboName = name
		self.hasData = False
		self.hasOdom = False
		rospy.Subscriber("/"+self.roboName+"/"+laserName, LaserScan, self.callback)
		rospy.Subscriber("/" + self.roboName + "/odom", Odometry, self.odomCallback)

	def callback(self, data):
		self.dataPack = data
		self.hasData = True
	
	def odomCallback(self, data):
		self.odomOrientation = data
		self.hasOdom = True

	def getOdomRotation(self):
		rotation = self.odomOrientation.pose.pose.orientation
		oList = [rotation.x, rotation.y, rotation.z, rotation.w]
		(roll, pitch, yaw) = euler_from_quaternion(oList)
		yaw *= 180/math.pi
		if yaw < 0:
			yaw += 360.0
		return yaw

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
	
	def isOdomAvail(self):
		return self.hasOdom

# class to claculate the specific state the robot should be in
class mazeFinderState:
	
	def __init__(self, name, laser):
		self.roboName = name
		self.laserName = laser
		self.sensorDataObj = sensorData(self.roboName, self.laserName)
		self.curState = STATES.WANDER
		self.ICONST = {"RIGHT":0, "FRONT":1, "LEFT":2}
		self.laserRanges = [0,0,0]
		self.dist = 2
		
	def updateState(self, myRanges, lenRanges):

		# get data
		angleIncrement = self.sensorDataObj.getAngleIncrement()
		angleMin = self.sensorDataObj.getAngleMin()
		rangeMax = self.sensorDataObj.getRangeMax()

		fIndex = self.calcIndex(0, angleMin, angleIncrement)
		rIndex = self.calcIndex(-89, angleMin, angleIncrement)
		lIndex = self.calcIndex(89, angleMin, angleIncrement)

		lower180 = rIndex
		upper180 = lIndex
		size = ((upper180 - lower180)/3)+1
		#print size

		ranges = []
		for x in myRanges:

			curRange = x
			if math.isinf(curRange):
				curRange = rangeMax
			ranges.append(curRange)

		#print ranges
		lower = rIndex
		upper = lower + size
		self.laserRanges[self.ICONST["RIGHT"]] = min(ranges[lower:upper])
		lower = upper+1
		upper = lower+size
		self.laserRanges[self.ICONST["FRONT"]] = min(ranges[lower:upper])
		lower = upper+1
		upper = lower+size
		self.laserRanges[self.ICONST["LEFT"]] = min(ranges[lower:upper])
		
		
		#logic for next move
		isRight = self.laserRanges[self.ICONST["RIGHT"]] <= self.dist
		isFront = self.laserRanges[self.ICONST["FRONT"]] <= self.dist
		isLeft = self.laserRanges[self.ICONST["LEFT"]] <= self.dist

		if isFront and isLeft and isRight: #111
			self.curState = STATES.RIGHT
		elif isFront and isLeft and not isRight: #110
			self.curState = STATES.RIGHT
		elif isFront and not isLeft and isRight: #101
			self.curState = STATES.RIGHT
		elif isFront and not isLeft and not isRight: #100
			self.curState = STATES.RIGHT
		elif not isFront and isLeft and isRight: #011
			self.curState = STATES.WANDER ## slight right
		elif not isFront and isLeft and not isRight: #010
			self.curState = STATES.FORWARD
		elif not isFront and not isLeft and isRight: #001
			self.curState = STATES.WANDER
		elif not isFront and not isLeft and not isRight: #000
			if self.curState == STATES.WANDER:
				self.curState = STATES.WANDER
			else:
				self.curState = STATES.CORNER
	def calcTheoretical(self, d, index, angleMin, angleIncrement):
		degree = (math.pi/2) - abs(self.calcTheta(index, angleMin, angleIncrement))
		result = d / math.cos(degree)
		return result

	def calcTheta(self, counter, angleMin, angleIncrement):
		return counter * angleIncrement + angleMin

	# enter angle in degrees
	def calcIndex(self, angle, angleMin, angleIncrement):
		nAngle = math.radians(angle)
		return int(math.floor((nAngle-angleMin)/angleIncrement))

	def determineMovement(self):
		hasData = self.sensorDataObj.isDataAvail()

		rTwist = Twist()
		if hasData == True:
			sensorRanges = self.sensorDataObj.getSensorData()
			self.updateState(sensorRanges, len(sensorRanges))

			if self.curState == STATES.FORWARD:
				#Forward
				rTwist = self.moveForward()
			elif self.curState == STATES.RIGHT:
				#Right
				rTwist = self.turnRight()
			elif self.curState == STATES.LEFT:
				#Left
				rTwist = self.turnLeft()
			elif self.curState == STATES.WANDER:
				#Wander
				rTwist = self.wander()
			elif self.curState == STATES.CORNER:
				rTwist = self.corner()


		#print rTwist
		return rTwist

	# All methods below are the different movements the robot needs to do in that state
	def moveForward(self):
		rTwist = Twist()
		rTwist.linear.x = 2
		return rTwist
	def turnRight(self):
		rTwist = Twist()
		rTwist.angular.z = -2
		return rTwist
	def turnLeft(self):
		rTwist = Twist()
		rTwist.angular.z = 2
		return rTwist
	def wander(self):
		rTwist = Twist()
		rTwist.linear.x = 3
		rTwist.angular.z = -2
		return rTwist
	def corner(self):
		rTwist = Twist()
		rTwist.linear.x = .5
		rTwist.angular.z = 2
		return rTwist



# class that handles the movement functions
class mazeFinderController:

	def __init__(self, name, laser):
		self.roboName = name
		self.laserName = laser
		
		self.pub = rospy.Publisher("/"+str(name)+"/cmd_vel", Twist, queue_size = 10)

		rospy.Subscriber("/"+name+"/wallLeft", Int32, self.wallLeftCallback)
		self.rate = rospy.Rate(10)
		self.mazeFinderStateObj = mazeFinderState(name, laser)
		self.isActive = False
	
	def wallLeftCallback(self, data):
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
