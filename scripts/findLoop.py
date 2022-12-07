#!/usr/bin/env python

import rospy
import math
from std_msgs.msg import String
from enum import Enum
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32
from unitysim.msg import BoundingBox3d
from nav_msgs.msg import Odometry

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
	def __init__(self, name):
		self.roboName = name
		self.hasOdomData = False
		rospy.Subscriber("/"+self.roboName+"/odom", Odometry, self.odomCallback)
		#print "Init"

	def odomCallback(self, data):
		self.odomData = data
		#print "Something"
		#print self.odomData
		self.hasOdomData = True

	def getOdomData(self):
		return self.odomData

	def hasData(self):
		return self.hasOdomData
		
	

# class that handles the movement functions
class mazeRunnerController:

	def __init__(self, name):
		self.roboName = name
		
		self.wallLeftPub = rospy.Publisher("/"+str(name)+"/loopDetect", Int32, queue_size = 10)

		self.rate = rospy.Rate(10)
		
		self.sensorDataObj = sensorData(self.roboName)

	def run(self):
		while not rospy.is_shutdown():
			if self.sensorDataObj.hasData():
				self.callState()
			self.rate.sleep()

	def callState(self):
		odomData = self.sensorDataObj.getOdomData()
		


	
# start of main
if __name__ == '__main__':
	rospy.init_node("Controller", anonymous=True)
	#robotName = rospy.get_param("~roboname")
	robotName = "jarboe2"
	try:
		mazeRunObj = mazeRunnerController(robotName)
		mazeRunObj.run()
	except rospy.ROSInterruptException:
		print("ERROR: something happened")
