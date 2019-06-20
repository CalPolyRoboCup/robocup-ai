import numpy as np
import math
import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from action import action
from MoveTo import MoveTo
from helper_functions import mag, convert_local, min_angle

'''
moves around the ball so that it faces target_loc
'''
class OrbitBall(MoveTo):
    def __init__(self, target_loc = False, offset = 125):
        MoveTo.__init__(self)
        self.target_loc = target_loc
        
        #how quickly the robot moves around the ball. Larger is slower
        self.spiral_factor = 0.15
        self.rot_spiral_factor = 0.825
        
        #if we are within speed_mod_distance of the ball push our target point out by
        #speed_mod_factor so that we orbit faster.
        self.speed_mod_factor = 7
        self.speed_mod_distance = 250
        
        #we look a bit behind the ball so as we orbit we will still be looking at the ball
        #this factor controls that
        self.rot_lead_factor = .75
        
        #number of seconds to lead the ball by
        self.extrapolation_factor = 0.0
        
        #how far from the ball we try to get
        self.offset = offset
    def add(self, robot, game):
        self.robot = robot
        MoveTo.add(self, robot, game)
    def run(self):
        '''
        we pull in to spiral_factor and rotate around by (1 - spiral_factor)
        we also push the target location out by speed_mod_factor if it is too close
            since the intermediate locations aren't where we want to stop
        '''
        
        #set target location to be closer to the ball
        ball_extrapolation = self.game.ball.loc + self.game.ball.velocity * self.extrapolation_factor
        robot_vec = self.robot.loc - ball_extrapolation
        robot_vec_scaled = robot_vec * self.offset / mag(robot_vec)
        target_loc = robot_vec_scaled * (1 - self.spiral_factor) + self.spiral_factor * robot_vec
        
        #get the angle between where we are relative to the ball and where we would like to be
        target_vec = ball_extrapolation - self.target_loc
        current_angle = -math.atan2(robot_vec[1], robot_vec[0])
        target_angle = math.atan2(-target_vec[1], target_vec[0])
        
        #rotate the vector from the first part by a fraction of the angle we are off by
        rotation_angle = -min_angle(current_angle - target_angle) * self.rot_spiral_factor
        orbit_vec = convert_local(target_loc, rotation_angle)
        
        #if we are close to the ball push target location out
        move_target = orbit_vec + ball_extrapolation
        speed_mod_vec = move_target - self.robot.loc
        if mag(speed_mod_vec) < self.speed_mod_distance:
                move_target = (move_target + speed_mod_vec * (abs(rotation_angle) + 
                                mag(self.game.ball.velocity)/300) * self.speed_mod_factor)
        
        #look a bit behind the ball so that as you orbit you are looking at the ball
        rot_vec = convert_local(target_loc, -rotation_angle) * self.rot_lead_factor
        point_dir = (ball_extrapolation + rot_vec) - self.robot.loc
        target_rot = -math.atan2(point_dir[1], point_dir[0])
        
        if mag(move_target - self.robot.loc) > 300:
                move_target += (move_target - self.robot.loc)
        
        #call pid code
        self.set_target(move_target, target_rot)
        actions = MoveTo.run(self)
        self.actions = actions
        return actions
