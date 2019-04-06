#!/usr/bin/env python

import rospy

from sslclient import SSLClient

from sim_receiver import SimReceiver

from std_msgs.msg import String

def ssl_vision_publisher():
    """
    Use python sockets library to interface with ssl vision
        - Look at mackennon's work in robocup-sim/simclient/robot & 
          robocup/simclient/communication/sim_receiver 
    """
    pub = rospy.Publisher('location', String, queue_size=1000)
    rospy.init_node('ssl_vision', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    rospy.loginfo("Sup")
    # our ssl client, provides us data from ssl-vision
    # ssl_client = sslclient.client()

    while not rospy.is_shutdown():
        rospy.loginfo("before")
        raw_data = ssl_client.receive()
        rospy.loginfo("after")
        rospy.loginfo(raw_data)
        rate.sleep()
        # data_object is placeholder for our object
        # create_msg is placeholder for method that generates custom message
        #custom_data = data_object.create_msg( raw_data )
        #if custom_data.HasField('geometry'):
        #    pass

if __name__== '__main__':
    try:
        ssl_client = SSLClient()
        ssl_client.connect()
        ssl_vision_publisher()
    except rospy.ROSInterruptException:
        pass 