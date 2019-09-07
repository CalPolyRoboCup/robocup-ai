import numpy as np
import math
import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from MoveTo import MoveTo
from helper_functions import mag, min_angle, angle_of, rotate_vector, angle_to, dist, scale_to, FIELD_HEIGTH

import time


class DribbleBall(MoveTo):
    def __init__(self, target_pos = None, offset = 140):
        '''
        Push the ball to a target location

        Preferably to be called after the ball has been captured. It 
        can capture the ball, just not as well as InterceptBall
        '''
        MoveTo.__init__(self)
        self.moving_to = False
        self.target_pos = target_pos

        self.offset = offset
        self.on_angle_margin = 0.8
        self.on_angle_factor = 3
        self.stop_radius = 300
        self.ball_lead_factor = 0.2
        self.spiral_factor = 1
        self.quick_turn_factor = 2

        #don't dribble the ball to the edge of the field or you won't have room to turn around
        self.dribble_bounds = 500

    def add(self, robot, game):
        MoveTo.add(self, robot, game)

    def set_target(self, target_pos):
        if target_pos[1] > FIELD_HEIGTH - self.dribble_bounds:
            target_pos[1] = FIELD_HEIGTH - self.dribble_bounds
        elif target_pos[1] < -FIELD_HEIGTH + self.dribble_bounds:
            target_pos[1] = -FIELD_HEIGTH + self.dribble_bounds
        self.target_pos = target_pos

    def run(self):
        #don't try accounting for velocity while we are controling the ball
        #feedback loops ensue.
        ball_extrapolation = self.game.ball.loc
        inside_stop_radius = dist(ball_extrapolation, self.robot.loc) > self.offset
        if inside_stop_radius:
            ball_extrapolation = ball_extrapolation + self.game.ball.velocity * self.ball_lead_factor
        
        #rotate towards the target. Limit rotation to maintain control.
        target_vec = ball_extrapolation - self.target_pos
        robot_vec = self.robot.loc - ball_extrapolation
        current_angle = angle_of(robot_vec)
        target_angle = angle_of(target_vec)
        error_angle = np.clip(min_angle(current_angle - target_angle), -self.spiral_factor, self.spiral_factor)

        #move in towards the ball
        off_angle_factor = abs(error_angle)/np.pi
        target_offset = self.offset * (1 - off_angle_factor) + self.quick_turn_factor*mag(robot_vec) * off_angle_factor
        target_loc = scale_to(robot_vec, target_offset)

        #apply the above operations
        orbit_vec = rotate_vector(target_loc, -error_angle)

        on_angle = self.on_angle_margin - abs(error_angle)
        if dist(self.robot.loc, self.target_pos) > self.stop_radius:
            if on_angle > 0:
                # if we are lined up and not at the target. Push quickly
                orbit_vec = orbit_vec - robot_vec * on_angle * self.on_angle_factor
        
        target_loc = orbit_vec + ball_extrapolation
        target_rot = angle_to(ball_extrapolation, self.robot.loc)
        MoveTo.set_target(self, target_loc, target_rot)
        return MoveTo.run(self)
