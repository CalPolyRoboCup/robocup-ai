#!/usr/bin/env python

import rospy
from robocup_msgs.msg import *
from std_msgs.msg import String

def coach():
	rospy.init_node("coach")

	#Coach node subscribes to the locations topic
	rospy.Subscriber("locations", Position, callback)

	rospy.Subscriber("test", String, callback2)

	# Coach node publishes to the strategy topic
	pub = rospy.Publisher("strategy", Strategy, queue_size=10)
	rate = rospy.Rate(10) # 10 Hz

	#Define the Strategy message
	msg = Strategy()
	msg.strategy = 0
	msg.additional_rules = 0

	#Publish the Strategy message to the strategy topic
	while not rospy.is_shutdown():
		#rospy.loginfo(msg)
		pub.publish(msg)
		rate.sleep()
	

#Process the Position message from the locations topic
def callback(Position):
	rospy.loginfo(Position.poses, Position.twists)

def callback2(data):
	rospy.loginfo(data)

if __name__ == '__main__':
	try:
		coach()
	except rospy.ROSInterruptException:
		pass