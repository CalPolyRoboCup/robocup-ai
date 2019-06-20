import numpy as np
import math
import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from MoveTo import MoveTo
from helper_functions import mag, min_angle, angle_of, rotate_vector, angle_to, dist, scale_to

import time


class DribbleBall(MoveTo):
    def __init__(self, target_pos = None, offset = 135):
        '''
        Push the ball to a target location

        Preferably to be called after the ball has been captured. It 
        can capture the ball, just not as well as InterceptBall
        '''
        MoveTo.__init__(self)
        self.moving_to = False
        self.target_pos = target_pos
        self.spiral_factor = .37
        self.near_spiral_factor = 0.0
        self.chase_down_factor = 0
        self.speed_mod_factor = 22
        self.offset = offset
        self.rot_lead_factor = 1

        self.on_angle_margin = 0.8
        self.on_angle_factor = 1.5
        self.off_angle_factor = 2.5
        self.epsilon = 50
        self.stop_radius = 240
        self.max_turn_angle = np.pi/4
        self.ball_lead_factor = 0.05

    def add(self, robot, game):
        MoveTo.add(self, robot, game)

    def set_target(self, target_pos):
        self.target_pos = target_pos

    def run(self):
        #don't try accounting for velocity while we are controling the ball
        #feedback loops ensue.
        ball_extrapolation = self.game.ball.loc
        if dist(ball_extrapolation, self.robot.loc) > self.offset + self.epsilon:
            ball_extrapolation = ball_extrapolation + self.game.ball.velocity * self.ball_lead_factor

        #move in from current location by spiral factor.
        robot_vec = self.robot.loc - ball_extrapolation
        robot_vec_scaled = scale_to(robot_vec, self.offset)
        target_loc = robot_vec_scaled * (1 - self.spiral_factor) + self.spiral_factor * robot_vec
        
        #rotate towards the target. Limit rotation to maintain control.
        target_vec = ball_extrapolation - self.target_pos
        current_angle = angle_of(robot_vec)
        target_angle = angle_of(target_vec)
        error_angle = min_angle(current_angle - target_angle)
        error_angle = np.clip(error_angle, -self.max_turn_angle, self.max_turn_angle)
        orbit_vec = rotate_vector(target_loc, -error_angle)

        on_angle = self.on_angle_margin - abs(error_angle)
        # if we are lined up and not at the target let em rip (go fast)
        if on_angle > 0 and dist(self.robot.loc, self.target_pos) > self.stop_radius:
            orbit_vec = orbit_vec - robot_vec * on_angle * self.on_angle_factor

        # if we are not lined up spin faster
        if on_angle < 0:
            orbit_vec = orbit_vec*(1 - on_angle * self.off_angle_factor) - robot_vec

        target_loc = orbit_vec + ball_extrapolation
        target_rot = angle_to(ball_extrapolation, self.robot.loc)
        MoveTo.set_target(self, target_loc, target_rot)
        return MoveTo.run(self)
