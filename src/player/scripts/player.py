#!/usr/bin/env python

import rospy
import message_filters
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from robocup_msgs.msg import Position
from robocup_msgs.msg import Strategy

class player():
    def __init__(self, num_bots=6):
        self.num = num_bots
        self.our_team = []
        self.other_team = []
        for i in range(num_bots):
            self.our_team.append(robot(None, None, None, i))
            self.other_team.append(robot(None, None, None, i))
        self.listener()

    def stratCallback(self, data):
        for r in self.our_team:
            r.updateStrat(data.strategy) 

    def posCallback(self, data):
        for i in range(self.num):
            rospy.loginfo("%d %s %s" % (i, data.poses[i], data.twists[i]))
            self.our_team[i].updateVals(data.poses[i].position, data.twists[i].linear, data.twists[i].angular)
            self.other_team[i].updateVals(data.poses[i+6].position, data.twists[i+6].linear, data.twists[i+6].angular)

    def listener(self):
        rospy.init_node('players_listener', anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.Subscriber('locations', Position, self.posCallback)
            rospy.Subscriber('coach', Strategy, self.stratCallback)
            rate.sleep()
        rospy.spin()

# class player():
#     def __init__(self, num_bots=6):
#         self.our_team = []
#         self.other_team = []
#         for i in range(num_bots):
#             self.our_team.append(robot(None, None, None))
#             self.other_team.append(robot(None, None, None))

#     def update_robot(self, data):
#         rospy.loginfo("updating data for robot %s", data.data)

#     def update_strat(self, data):
#         rospy.loginfo("update strat! %s", data.data)

#     def listener(self):
#         rate = rospy.Rate(10)
#         rospy.init_node('listener', anonymous=True)

#         rospy.Subscriber('ssl', Position, self.update_robot)
#         rospy.Subscriber('coach', Strategy, self.update_strat)

#         rospy.spin()

class robot():
    def __init__(self, pos, vel, angl, id):
        self.pos = pos
        self.vel = vel
        self.angl = angl
        self.id = id
        self.strat = None

    def getPos(self):
        return self.pos

    def getVel(self):
        return self.vel
    
    def getAngl(self):
        return self.angl

    def getId(self):
        return self.id

    def getStrat(self):
        return self.strat

    def updateID(self, newID):
        self.id = newID

    def updateVals(self, newPos, newVel, newAngl):
        if newPos != None:
            self.pos = newPos
        if newVel != None:
            self.vel = newVel
        if newAngl != None:
            self.angl = newAngl

    def updateStrat(self, strat):
        self.strat = strat

    def __str__(self):
        return "%d %s %s %s" % (self.id, self.pos, self.vel, self.angl)

if __name__ == '__main__':
    try:
	    p = player() 
    except rospy.ROSInterruptException:
        pass