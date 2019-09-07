import numpy as np
import sys
import os
import math
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from action import action
from DribbleBall import DribbleBall
from helper_functions import mag, put_in_bounds, angle_of

class KickTo(DribbleBall):
    def __init__(self, kick_target = None, deliberation_time = 13, epsilon = 0.05):
        DribbleBall.__init__(self)
        #don't push the ball
        self.on_angle_factor = 0.1

        # maximum angle in radians between current kick vector and target kick vector that
        # is acceptable for a pass
        self.pass_epsilon = epsilon
        
        # we wait a few cycles once the ball is in position
        # to ensure the ball alignment with the kicker is good
        self.deliberation = 0
        self.deliberation_time = deliberation_time
        
    def run(self):
        
        tvec = self.target_pos - self.robot.loc
        bvec = self.robot.facing#self.game.ball.loc - self.robot.loc
        # if we have the ball in position
        if abs(angle_of(bvec) - angle_of(tvec)) < self.pass_epsilon and self.game.ball.controler != False:
            # wait a bit then kick
            self.deliberation -= 1
            if self.deliberation <= 0:
                self.kick = 1
                self.deliberation = self.deliberation_time
        else:
            # otherwise don't kick
            self.kick = 0
            self.deliberation = self.deliberation_time
            
        # turn and face the target loc
        # note this is dependent on self.target_loc being set
        actions = DribbleBall.run(self)
        return actions
        
    def done(self):
        # this will return true when the ball has been kicked
        # TODO: This is jank. I think the jank is just for the test
        if self.kick:
            self.kick = 0
            return 1
        return 0


LEAD_FACTOR = 2E-4
class PassTo(KickTo):
    def __init__(self, target_robot = None, deliberation_time = 6, epsilon = 0.1, lead_factor = LEAD_FACTOR):
        KickTo.__init__(self, None, deliberation_time, epsilon)
        self.target_robot = target_robot

        # how much we lead the target we are kicking to
        self.lead_factor = lead_factor
        
    def run(self):
        # lead the target we are passing to
        velocity_mod = self.target_robot.velocity * self.lead_factor * mag(self.target_robot.loc - self.robot.loc)
        target_pos = self.target_robot.loc + velocity_mod
        self.target_pos = target_pos
        self.target_pos, _ = put_in_bounds(self.target_pos)
        return KickTo.run(self)