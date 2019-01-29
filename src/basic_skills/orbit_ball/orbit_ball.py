import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '../..')
from basic_skills.action import *
from basic_skills.move_to.move_to import *
from basic_skills.helper_functions import *
from matplotlib.widgets import Slider, Button, RadioButtons
from pygame_simulator.PySim_noise import *

'''
moves around the ball so that it faces target_loc
'''
class orbit_ball(action):
  def __init__(self, target_loc = False, offset = 125):
    action.__init__(self)
    self.pid = move_to()
    self.move_to = False
    self.target_loc = target_loc
    
    #how quickly the robot moves around the ball. Larger is slower
    self.spiral_factor = 0.35
    self.rot_spiral_factor = 0.825
    
    #if we are within speed_mod_distance of the ball push our target point out by
    #speed_mod_factor so that we orbit faster.
    self.speed_mod_factor = 7
    self.speed_mod_distance = 300
    
    #we look a bit behind the ball so as we orbit we will still be looking at the ball
    #this factor controls that
    self.rot_lead_factor = .35
    
    #number of seconds to lead the ball by
    self.extrapolation_factor = 0.1
    
    #how far from the ball we try to get
    self.offset = offset
  def add(self, robot, game):
    self.robot = robot
    self.pid.robot = robot
    action.add(self, robot, game)
  def run(self):
    '''
    we pull in to spiral_factor and rotate around by (1 - spiral_factor)
    we also push the target location out by speed_mod_factor if it is too close
      since the intermediate locations aren't where we want to stop
    '''
    
    #set target location to be closer to the ball
    ball_extrapolation = self.game.ball.loc + self.game.ball.velocity * self.extrapolation_factor
    robot_vec = self.robot.loc - ball_extrapolation
    robot_vec_scaled = robot_vec * self.offset / np.linalg.norm(robot_vec)
    target_loc = robot_vec_scaled * (1 - self.spiral_factor) + self.spiral_factor * robot_vec
    
    #get the angle between where we are relative to the ball and where we would like to be
    target_vec = ball_extrapolation - self.target_loc
    current_angle = -math.atan2(robot_vec[1], robot_vec[0])
    target_angle = math.atan2(-target_vec[1], target_vec[0])
    
    #rotate the vector from the first part by a fraction of the angle we are off by
    rotation_angle = -min_angle(current_angle - target_angle) * self.rot_spiral_factor
    orbit_vec = convert_local(target_loc, rotation_angle)
    
    #if we are close to the ball push target location out
    move_to = orbit_vec + ball_extrapolation
    speed_mod_vec = move_to - self.robot.loc
    if np.linalg.norm(speed_mod_vec) < self.speed_mod_distance:
      move_to = move_to + speed_mod_vec * abs(rotation_angle) * self.speed_mod_factor
    
    #look a bit behind the ball so that as you orbit you are looking at the ball
    rot_vec = convert_local(target_loc, -rotation_angle) * self.rot_lead_factor
    point_dir = (ball_extrapolation + rot_vec) - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    
    #call pid code
    self.pid.set_target(move_to, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.moving_to = move_to
    return actions
