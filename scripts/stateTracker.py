#!/usr/bin/env python
import rospy
import math
from std_msgs.msg import String
from enum import Enum
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from unitysim.msg import BoundingBox3d


class myTimer:

	def __init__(self, duration):
		self.seconds = 0
		self.callBackTimeout = duration


	def getDuration(self):
		return self.seconds

	def resetTimer(self):
		self.seconds = 0


	def timerCallback(self, event):
		self.seconds += self.callBackTimeout
			
	def startTimer(self):
		rospy.Timer(rospy.Duration(self.callBackTimeout), self.timerCallback)



class sensorData:
	def __init__(self, name, laserName):
		self.roboName = name
		self.hasData = False
		self.hasHealthFinderData = False
		self.hasPlayerFinderData = False

		rospy.Subscriber("/"+self.roboName+"/"+laserName, LaserScan, self.callback)
		rospy.Subscriber("/"+self.roboName+"/healthfinder", BoundingBox3d, self.healthFinderCallback)
		rospy.Subscriber("/"+self.roboName+"/playerfinder", BoundingBox3d, self.playerFinderCallback)
		rospy.Subscriber("/"+self.roboName+"/hp", Int32, self.hpCallBack)

		self.curHP = 5

		self.pointPub = rospy.Publisher("/"+self.roboName+"/closestPoint", BoundingBox3d, queue_size = 10)
		self.playerPub = rospy.Publisher("/"+self.roboName+"/closestPlayer", BoundingBox3d, queue_size = 10)
		
		self.mTimer = myTimer(.01)
		self.mTimer.startTimer()
		self.lastHFCheck = -9999
		self.lastPFCheck = -9999
	
	def healthFinderCallback(self, data):
		if not self.hasHealthFinderData:
			self.healthFinderData = data
			self.lastHFCheck = self.mTimer.getDuration()
		else:
			if self.isHealthFinderInView():
				if self.getHFSize(self.healthFinderData) <= self.getHFSize(data):
					self.healthFinderData = data
					self.lastHFCheck = self.mTimer.getDuration()
				else:
					pass # at this point there is a bigger blob alerady in view so go to the one first
			else:
				self.healthFinderData = data
				self.lastHFCheck = self.mTimer.getDuration()
		self.pointPub.publish(self.healthFinderData)
		self.hasHealthFinderData = True

	def playerFinderCallback(self, data):
		if not self.hasPlayerFinderData:
			self.playerFinderData = data
			self.lastPFCheck = self.mTimer.getDuration()
		else:
			if self.isPlayerFinderInView():
				if self.getPFSize(self.playerFinderData) <= self.getPFSize(data):
					self.playerFinderData = data
					self.lastPFCheck = self.mTimer.getDuration()
				else:
					pass # at this point there is a bigger blob alerady in view so go to the one first
			else:
				self.playerFinderData = data
				self.lastPFCheck = self.mTimer.getDuration()
		self.playerPub.publish(self.playerFinderData)
		self.hasPlayerFinderData = True
	def callback(self, data):
		self.dataPack = data
		self.hasData = True

	def hpCallBack(self, data):
		self.curHP = data
		
	
	#region sensor data
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
	#endregion

	#region HP
	def getHP(self):
		return self.curHP
	#endregion

	#region healthFinderData
	def isHealthFinderInView(self):
		curTime = self.mTimer.getDuration()
		diff = curTime - self.lastHFCheck
		if diff > 1:
			return False
		else:
			return True


	def getHFRotation(self):
		return self.healthFinderData.position.y

	def getHFSize(self, data):
		# need to figure out how the sizes work
		return data.size.y * data.size.z

	#endregion
	def isHealthFinderDataAvail(self):
		return self.hasHealthFinderData

	#region playerFinderData
	def isPlayerFinderInView(self):
		curTime = self.mTimer.getDuration()
		diff = curTime - self.lastPFCheck
		if diff > 1:
			return False
		else:
			return True


	def getPFRotation(self):
		return self.playerFinderData.position.y

	def getPFSize(self, data):
		# need to figure out how the sizes work
		return data.size.y * data.size.z

	#endregion
	def isPlayerFinderDataAvail(self):
		return self.hasPlayerFinderData

# class that handles the movement functions
class battleBotController:

	def __init__(self, name, laser):
		self.roboName = name
		self.laserName = laser
		
		self.wallLeftPub = rospy.Publisher("/"+str(name)+"/wallLeft", Int32, queue_size = 10)
		self.wallRightPub = rospy.Publisher("/"+str(name)+"/wallRight", Int32, queue_size = 10)
		self.mostOpenPathPub = rospy.Publisher("/"+str(name)+"/mostOpenPath", Int32, queue_size = 10)
		self.grabObjectPub = rospy.Publisher("/"+str(name)+"/grabObject", Int32, queue_size = 10)
		self.attackPlayerPub = rospy.Publisher("/"+str(name)+"/attackPlayer", Int32, queue_size = 10)
		self.rate = rospy.Rate(10)
		
		self.sensorDataObj = sensorData(self.roboName, self.laserName)

	def run(self):
		while not rospy.is_shutdown():
			if self.sensorDataObj.isDataAvail():
				self.callState()
			self.rate.sleep()

	def callState(self):
		ranges = self.sensorDataObj.getSensorData()
		angleIncrement = self.sensorDataObj.getAngleIncrement()
		angleMin = self.sensorDataObj.getAngleMin()
		rangeMax = self.sensorDataObj.getRangeMax()

		# set basic variables
		horzSum = 0
		vertSum = 0
		counter = 0

		length = len(ranges)
		forwardRange = length/3
		partialForwardRange = forwardRange/2
		middle = length/2
		upperVert = middle + partialForwardRange
		lowerVert = middle - partialForwardRange

		for x in ranges:
			curRange = x
			if math.isinf(curRange):
				curRange = rangeMax
			theta = counter * angleIncrement + angleMin
			curRange /= rangeMax
			#vertSum += self.calcVert(curRange, theta) # Forward Vector
			horzSum += self.calcHorz(curRange, theta) # Rotational Vector 1

			counter += 1
		#vertSum /= len(myRanges)
		horzSum /= len(ranges)


		minVert = min(ranges[lowerVert:upperVert+1]) # / to get between 0 and 1
		if minVert <= 2:
			if horzSum >= 0:
				#self.startWallLeft()
				self.startWallRight()
				rospy.sleep(.2)
				#print("Wall Right")
			else:
				self.startWallLeft()
				rospy.sleep(.2)
				#self.startWallRight()
				#print("Wall Left")
		else:
			if self.sensorDataObj.isHealthFinderInView() and not self.sensorDataObj.isPlayerFinderInView():
				self.startGrabPoint()
				#print("Grab Point")
				#print("OO I FOUND AN OBJECT TO GRAB")
			elif not self.sensorDataObj.isHealthFinderInView() and self.sensorDataObj.isPlayerFinderInView():
				self.startAttackPlayer()
			elif self.sensorDataObj.isHealthFinderInView() and self.sensorDataObj.isPlayerFinderInView():
				if self.sensorDataObj.getHP() <= 2:
					self.startGrabPoint()
				else:
					self.startAttackPlayer()
			else:
				self.startOpenFollow()
				#print("Open Follow")
			
		#self.wallLeftPub.publish(Int32(0))
		#self.wallRightPub.publish(Int32( 0))
		#self.mostOpenPathPub.publish(Int32(1))

	def calcHorz(self, num, theta):
		return math.sin(theta) * num
	def calcVert(self, num, theta):
		return math.cos(theta) * num
	
	def startWallLeft(self):
		self.wallRightPub.publish(Int32(0))
		self.mostOpenPathPub.publish(Int32(0))
		self.grabObjectPub.publish(Int32(0))
		self.attackPlayerPub.publish(Int32(0))
		self.wallLeftPub.publish(Int32(1))
		print("STATE: wall left")

	def startWallRight(self):
		self.wallLeftPub.publish(Int32(0))
		self.mostOpenPathPub.publish(Int32(0))
		self.grabObjectPub.publish(Int32(0))
		self.attackPlayerPub.publish(Int32(0))
		self.wallRightPub.publish(Int32(1))
		print("STATE: wall right")

	def startOpenFollow(self):
		self.wallLeftPub.publish(Int32(0))
		self.wallRightPub.publish(Int32(0))
		self.grabObjectPub.publish(Int32(0))
		self.attackPlayerPub.publish(Int32(0))
		self.mostOpenPathPub.publish(Int32(1))
		print("STATE: open follow")

	def startGrabPoint(self):
		self.wallLeftPub.publish(Int32(0))
		self.wallRightPub.publish(Int32(0))
		self.mostOpenPathPub.publish(Int32(0))
		self.attackPlayerPub.publish(Int32(0))
		self.grabObjectPub.publish(Int32(1))
		print("STATE: grab point")
	
	def startAttackPlayer(self):
		self.wallLeftPub.publish(Int32(0))
		self.wallRightPub.publish(Int32(0))
		self.mostOpenPathPub.publish(Int32(0))
		self.grabObjectPub.publish(Int32(0))
		self.attackPlayerPub.publish(Int32(1))
		print("STATE: ATTACK PLAYER")

	
# start of main
if __name__ == '__main__':
	rospy.init_node("Controller", anonymous=True)
	robotName = rospy.get_param("~roboname")
	laserName = rospy.get_param("~lasername")
	try:
		battleBotObj = battleBotController(robotName, laserName)
		battleBotObj.run()
	except rospy.ROSInterruptException:
		print("ERROR: something happened")
