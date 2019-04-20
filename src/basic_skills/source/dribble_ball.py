import numpy as np
import math
import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from move_to import *
from helper_functions import *
'''
Move the ball to a target_loc
Preferably to be called after the ball has been captured, but it 
can capture the ball, just not as well as ball_interception

This code is closely related to orbit_ball
'''
class dribble_ball(action):
  def __init__(self, target_loc = None, offset = 90):
    action.__init__(self)
    self.pid = move_to()
    self.moving_to = False
    self.target_loc = target_loc
    self.target_rot = None
    self.spiral_factor = .37
    self.near_spiral_factor = 0.0
    self.chase_down_factor = 0
    self.speed_mod_factor = 22
    self.offset = offset
    
    #the robot turns ahead of where the ball currently is
    #making this larger makes the robot lead more
    #this helps improve turning rate
    self.rot_lead_factor = .8
    
    #The robot accelerates when ready to push the ball
    #larger values make this condition harder to satisfy in terms of angle
    self.on_angle_tolerance = 2
    
    #The robot accelerates when ready to push the ball
    #This is the distance between the ball and robot where this acceleration starts
    self.scaling_distance = 200
    
    #The robot accelerates when ready to push the ball
    #larger values of this increase the magnitude of that acceleration
    self.push_speed_factor = 0.4
    
    #The robot accelerates when ready to push the ball
    #This motion is often erratic so we smooth it
    self.smoothing_factor = 0.9
    
  def set_target(self, target_loc, target_rot):
    self.target_loc = target_loc
    self.target_rot = target_rot
  def add(self, robot, game):
    #print("2999")
    self.pid.add(robot, game)
    action.add(self, robot, game)
  def run(self):
    #pull from current location in by spiral factor
    ball_extrapolation = self.game.ball.loc
    robot_vec = self.robot.loc - ball_extrapolation
    robot_vec_scaled = robot_vec * self.offset / np.linalg.norm(robot_vec)
    target_loc = robot_vec_scaled * (1 - self.spiral_factor) + self.spiral_factor * robot_vec
    
    #get desired global angle and use the difference between that and current angle to rotate "target_loc"
    #by a scaled amount
    target_vec = ball_extrapolation - self.target_loc
    current_angle = -math.atan2(robot_vec[1], robot_vec[0])
    target_angle = -math.atan2(target_vec[1], target_vec[0])
    rotation_angle = -min_angle(current_angle - target_angle) * (1 - self.spiral_factor)
    orbit_vec = convert_local(target_loc, rotation_angle)
    
    
    move_to = orbit_vec + ball_extrapolation
    on_angle_factor = np.clip(math.pi - self.on_angle_tolerance*abs(rotation_angle), 0, math.pi)**2
    if (np.linalg.norm(self.robot.loc - self.game.ball.loc) < self.scaling_distance):
      move_to = (move_to + (self.game.ball.loc - self.robot.loc)*(on_angle_factor)*self.push_speed_factor)
      move_to = move_to * (1 - self.smoothing_factor) + self.smoothing_factor * self.moving_to

    #look at the ball and lead by an amount
    #call pid
    rot_vec = convert_local(target_loc, -rotation_angle) * self.rot_lead_factor
    point_dir = (ball_extrapolation + rot_vec) - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    self.pid.set_target(move_to, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.moving_to = move_to
    return actions
