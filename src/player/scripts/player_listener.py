#!/usr/bin/env python

import sys
sys.path.insert(0, "/home/adleywong/catkin_ws/src/robocup_ai/player/src")
from player import Player

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from robocup_msgs.msg import Position
from robocup_msgs.msg import Strategy

p = Player()

def strat_callback(data):
    p.update_strategy(data)

def pos_callback(data):
    p.update_positions(data)
    
def run():
    rospy.init_node('players', anonymous=True)
    rospy.Subscriber('locations', Position, pos_callback)
    rospy.Subscriber('strategy', Strategy, strat_callback)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    try:
	    run()
    except rospy.ROSInterruptException:
        pass