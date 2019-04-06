#!/usr/bin/env python

import sim_receiver
import rospy
from std_msgs import String

def chatter_callback(message)
    rospy.logininfo(rospy.get_caller_id() + "ssl reports %s", message.data)

def listener()
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('chatter', String, chatter_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()