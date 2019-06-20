import numpy as np
import sys
import os
import math
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from action import action
from OrbitBall import OrbitBall
from helper_functions import mag, put_in_bounds, normalize_angle

class KickTo(OrbitBall):
    def __init__(self, kick_target = None, offset = 125, epsilon = 0.06):
        OrbitBall.__init__(self, None, offset)
        self.kick_target = kick_target
        
        # maximum angle in radians between current kick vector and target kick vector that
        # is acceptable for a pass
        self.epsilon = epsilon
        
        # we wait a few cycles once the ball is in position
        # to ensure the ball alignment with the kicker is good
        self.deliberation = 0
        self.deliberation_time = 15
        
    def run(self):
        self.target_loc = self.kick_target
        
        tvec = self.target_loc - self.robot.loc
        bvec = self.game.ball.loc - self.robot.loc
        
        # if we have the ball in position
        if abs(normalize_angle(math.atan2(tvec[1], tvec[0]) - math.atan2(bvec[1], bvec[0]))) < self.epsilon and self.game.ball.controler != False:
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
        actions = OrbitBall.run(self)
        return actions
        
    def done(self):
        # this will return true when the ball has been kicked
        # TODO: This is jank. I think the jank is just for the test
        if self.kick:
            self.kick = 0
            return 1
        return 0

class PassTo(KickTo):
    def __init__(self, target_robot = None):
        KickTo.__init__(self)
        self.target_robot = target_robot

        # how much we lead the target we are kicking to
        self.lead_factor = 0.0004
        
    def run(self):
        # lead the target we are passing to
        velocity_mod = self.target_robot.velocity * self.lead_factor * mag(self.target_robot.loc - self.robot.loc)
        self.kick_target = self.target_robot.loc + velocity_mod
        return KickTo.run(self)