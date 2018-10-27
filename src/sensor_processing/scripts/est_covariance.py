#!/usr/bin/env python

import rospy
import sys

from sensor_msgs.msg import Imu

global Imu sample_data[5]

class est_covariance:
    def __init__(self):
        self.pub = rospy.Publisher('imu_fixed', Imu, queue_size = 10)
        self.sub = rospy.Subscriber('imu', Imu, self.estimate, queue_size=1)

    def estimate(self, data):
        Imu imu_data = data.data
        #compute a sample covariance and fill out matrix with the 5 last samples
        

def main(args):
    est = est_covariance()
    rospy.init_node('est_cov', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'shutting down converter'
        

if __name__ == '__main__':
    main(sys.argv)
