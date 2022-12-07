#!/usr/bin/env python

import rospy


from std_msgs.msg import String

# start of main
if __name__ == '__main__':
	pub = rospy.Publisher("unity_spawn", String, queue_size = 10)
	rospy.init_node("spawnrobot", anonymous=True)
	robotName = rospy.get_param("~roboname")
	rate = rospy.Rate(10)
	try:
		pub.publish(robotName)
		rate.sleep()
		while not rospy.is_shutdown():
			pass
		print("Spawn robot with name " + robotName)
	except rospy.ROSInterruptException:
		print("ERROR: something happened")
