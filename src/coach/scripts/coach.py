#!/usr/bin/env python

import rospy

def coach():
	pub = rospy.Publisher('strategy', Strategy, queue_size=10)
	#Topic: strategy
	#Message type: Strategy (exists in msg but not currently defined)
	rospy.init_node('coach', anonymous = True)
	rospy.Subscriber('locations', GameData, callback)
	rate = rospy.Rate(10) #10 Hz
	while not rospy.is_shutdown():
		currentStrat = None #temp
		rospy.loginfo(currentStrat)
		pub.publish(currentStrat)
		rate.sleep()
	rospy.spin()

if __name__ == '__main__':
	try:
		coach()
	except rospy.ROSInterruptException:
		pass