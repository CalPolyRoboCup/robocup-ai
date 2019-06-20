import numpy as np
import math
import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/../..')
from MoveTo import MoveTo
from helper_functions import mag, drop_perpendicular, angle_to
from ball import ball
from robot import robot


class cover(MoveTo):
    '''
    covers a pass from target1 to target 2
        target1 - first target (robot, ball, or np.array)
        target2 - second target (robot, ball, or np.array)
        interpose_factor - where to position between target1 and target2. Range 0-1 (0 on top of target1, 1 on top of target2)
        interpose_weight - trade off between getting to pass line and getting to interpose_factor positioning
        lead_factor - if targets are robots or ball how many frames to extend their current movement
    '''
    def __init__(self, target1, target2, interpose_factor = .5, interpose_weight = 25, lead_factor = 0.3):
        MoveTo.__init__(self)
        
        self.target1 = target1
        self.target2 = target2
        
        self.interpose_factor = interpose_factor
        self.interpose_weight = interpose_weight  
        self.lead_factor = lead_factor
        
    def add(self, robot, game):
        MoveTo.add(self, robot, game)
        
    def parse_target(self, target):
        '''
        input 
        target - robot, ball, or np.array
        returns
        target_loc - np.array of target with velocity accounted for if necicary
        '''
        try:
            target_loc = target.loc
            if self.lead_factor:
                target_loc = target_loc + target.velocity * self.lead_factor
            #TODO add rotation accomadations
        except AttributeError:
            target_loc = target
        return target_loc
    
    
    def run(self):
        '''
        Move the robot into position to block a pass from target1 to target2
        '''
        target1_loc = self.parse_target(self.target1)
        target2_loc = self.parse_target(self.target2)
        shooting_vec = target2_loc - target1_loc

        #Find the point that will put you between the shooting position and the ball the quickest
        quickest_loc = drop_perpendicular(self.robot.loc, target1_loc, shooting_vec)
        interpose_loc = target1_loc + shooting_vec * self.interpose_factor
            
        #use our distance to the ball to average the two points (interpose_loc, quickest_loc) together
        quickest_distance = mag(quickest_loc - self.robot.loc)
        target_loc = ((quickest_loc * quickest_distance + interpose_loc * self.interpose_weight)
                                    / (self.interpose_weight + quickest_distance))
        
        #look at the ball
        target_rot = angle_to(target1_loc, self.robot.loc)
        
        #call PID
        MoveTo.set_target(self, target_loc, target_rot)
        return MoveTo.run(self)

    def done(self):
        return False
