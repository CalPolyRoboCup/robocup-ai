#!/usr/bin/env python
import math

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
from robocup_msgs.msg import Position
from robocup_msgs.msg import Strategy

from robot import Robot

#Player Node
class Player():
    def __init__(self, num_bots=6):
        self.num = num_bots
        self.our_team = []
        self.other_team = []
        self.ball = None
        self.closest_robot = None
        self.closest_dist_to_ball = 0xFFFFFFFFF
        
        for i in range(num_bots):
            self.our_team.append(Robot(None, None, None, i))
            self.other_team.append(Robot(None, None, None, i))

    def update_strategy(self, data):
        for r in self.our_team:
            rospy.loginfo("%d: %s" % (r.get_id(), data))
            r.update_strat(data.strategy) 

    def distance(self, p1, p2):
        return math.sqrt(math.pow(p1.x + p2.x, 2) 
            + math.pow(p1.y + p2.y, 2)
            + math.pow(p1.z + p2.z, 2))

    def is_closest(self, robot_pos):
        d = self.distance(self.ball, robot_pos)
        if d < self.closest_dist_to_ball:
            self.closest_dist_to_ball = d
            return True 
        return False
  
    def update_positions(self, data):
        self.ball = data.ball_pos
        for i in range(self.num):
            rospy.loginfo("%d %s %s %s" % (i, data.poses[i], data.twists[i], data.ball_pos))

            self.our_team[i].update_vals(data.poses[i].position, data.twists[i].linear, data.twists[i].angular)
            self.other_team[i].update_vals(data.poses[i+6].position, data.twists[i+6].linear, data.twists[i+6].angular)

            if(self.is_closest(data.poses[i].position)):
                self.closest_robot = self.our_team[i]
                
        rospy.loginfo("closest robot: %d\t ball pos: %s" % (self.closest_robot.get_id(), self.ball))
        self.closest_dist_to_ball = 0xFFFFFFFF