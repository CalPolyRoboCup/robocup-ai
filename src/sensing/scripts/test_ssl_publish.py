#!/usr/bin/env python

import sim_receiver
import rospy
from std_msgs import String

def ssl_talker():
    pub = rospy.Publisher('ssl_chatter', String, queue_size=10)
    rospy.init_node('ssl_talker',anonymous=True)
    rate =  rospy.Rate(1)

    while not rospy.is_shutdown():
        ssl_str = "SSL says: %s" %client.receive()
        rospy.loginfo(ssl_str)
        pub.publish(ssl_str)
        rate.sleep()

if __name__== '__main__':
    try:
        client = sim_receiver()
        client.open()
        ssl_talker()
    except rospy.ROSInterruptException:
        pass 