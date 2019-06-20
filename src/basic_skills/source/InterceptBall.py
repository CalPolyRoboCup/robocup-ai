import numpy as np
import math
import sys
import os
import time
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from action import action
from MoveTo import MoveTo
from OrbitBall import OrbitBall
from helper_functions import (mag, scale_to, dist, 
        min_angle, drop_perpendicular, normalize_angle,
        travel_time, rotate_vector, angle_of, mag_with,
        angle_to, FIELD_HEIGTH, FIELD_LENGTH)

# Attempts to gain control of the ball
# should be canceled once the ball is captured. This action is bad at maintaining control
# TODO make ball_intercept work for goalie. set plan_path = False
class InterceptBall(MoveTo):
    def __init__(self):
        MoveTo.__init__(self)
        
        #bounds for simple cases that can be solved easily
        self.slow_speed = 200
        
        #positioning constants
        self.offset = 115
        self.sweep_arch = math.pi/4
        self.sweep_radius = 2500
        self.spiral_factor = 0.3

    '''
    if the ball is moving slowly - move to it
    if the ball is comming towards us - get on the path to catch it
    if the ball is past us - chase after it
    '''
    def run(self):
        ball = self.game.ball
        target_rot = angle_to(ball.loc, self.robot.loc)

        # target_loc will be a wighted average of three locations
        # for three different tasks
        # weighted averaging ensures smooth transitions
        target_loc = np.array([0,0])
        loc_mag = 0

        # if the ball is moving towards us 
        # get in its path
        drop_loc = drop_perpendicular(self.robot.loc, ball.loc, ball.velocity)
        ball_closing_velocity = mag_with(drop_loc - ball.loc, ball.velocity)
        if ball_closing_velocity > 0:
            delay_time = travel_time(self.robot, drop_loc)
            target_loc = drop_loc
            target_loc = target_loc + ball.velocity * delay_time
            loc_mag += 1

        # if the ball got past us
        # sweep around it so we don't hit it
        dash_mag = (-ball_closing_velocity - self.sweep_radius) / self.sweep_radius
        if dash_mag > 0:
            sweep_vec = self.robot.loc - ball.loc
            target_vec = ball.velocity
            current_angle = angle_of(sweep_vec)
            target_angle = angle_of(target_vec)
            sweep_vec = rotate_vector(sweep_vec, self.spiral_factor*min_angle(current_angle - target_angle))
            sweep_mag = (1 - self.spiral_factor) * mag(sweep_vec) + self.spiral_factor * self.offset
            sweep_vec = scale_to(sweep_vec, sweep_mag)

            sweep_vec = scale_to(sweep_vec, sweep_mag)
            dash_loc = ball.loc + sweep_vec

            target_loc = target_loc + dash_loc * dash_mag
            loc_mag += dash_mag

        # if the ball is moving slowly
        # move directly to the ball
        grab_mag = self.slow_speed - mag(ball.velocity)
        if grab_mag > 0 or loc_mag == 0:
            grab_loc = ball.loc + scale_to(self.robot.loc - ball.loc, self.offset)
            grab_mag /= self.slow_speed
            target_loc = target_loc + grab_loc * grab_mag
            loc_mag += grab_mag

        target_loc = target_loc / loc_mag
        target_loc = put_in_bounds_on_line(target_loc, ball.velocity)
        self.set_target(target_loc, target_rot)
        return MoveTo.run(self)

# put location in bounds while staying on the specified line
# TODO handle Goalie box. 
def put_in_bounds_on_line(location, vector , push_in_factor = 1.1):
    if location[1] > FIELD_HEIGTH:
        scale = (location[1] - FIELD_HEIGTH) / vector[1] * push_in_factor
        location = location - vector * scale
        
    if location[1] < -FIELD_HEIGTH:
        scale = (location[1] + FIELD_HEIGTH) / vector[1] * push_in_factor
        location = location - vector * scale

    
    if location[0] > FIELD_LENGTH:
        scale = (location[0] - FIELD_LENGTH) / vector[0] * push_in_factor
        location = location - vector * scale
        
    if location[0] < -FIELD_LENGTH:
        scale = (location[0] + FIELD_LENGTH) / vector[0] * push_in_factor
        location = location - vector * scale
    return location

    