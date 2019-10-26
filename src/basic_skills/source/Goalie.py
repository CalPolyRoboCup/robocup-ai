import numpy as np
import sys
import os
import math
dirname = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, dirname)
from action import action
from MoveTo import MoveTo
from helper_functions import mag, angle_to, scale_to


class Goalie(MoveTo):
    def __init__(self):
        MoveTo.__init__(self)
        self.goal = np.array([0,0])
        self.orbit = 1500

        #turn off polite pathing
        self.plan_path = False
        
    def add(self, robot, game):
        MoveTo.add(self, robot, game)
        if self.robot.is_blue:
            self.goal = np.array([-6000,0])
        else:
            self.goal = np.array([6000,0])
            
    def push_out_function(self, dist):
        '''
        brief: we want to move forward to intercept the ball and cover more of the goal.
                        This bit of math squashes the distance from the ball to the goal so that 
                        if the ball is close to the goal a large number is returned. Otherwise a
        params: dist - distance from ball to goal 
        returns: value - squashed value as described above.
        '''
        #small number.
        return 1500000/(100 + dist)
    
    def run(self):
    
        adjustment = self.push_out_function(mag(self.game.ball.loc - self.goal))
        
        #recoil to buy time and absorb shock if the ball is free
        # if self.game.ball.controler == False:
            # adjustment = -adjustment
            
        #we will position ourselves between the ball and the goal 
        push_out_distance = self.orbit + adjustment
        push_vec = self.game.ball.loc - self.goal
        
        #if the ball is inside our current radius go in to it
        if push_out_distance > mag(push_vec):
            push_out_distance = mag(push_vec) - 40
        push_vec = scale_to(push_vec, push_out_distance)

        target_loc = self.goal + push_vec
        target_rot = angle_to(self.game.ball.loc, self.robot.loc)
        
        self.set_target(target_loc, target_rot)
        return MoveTo.run(self)
