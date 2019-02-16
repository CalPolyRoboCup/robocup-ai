import numpy as np
import math
import sys
from basic_skills.action import *
from basic_skills.move_to import *
from basic_skills.helper_functions import *

robot_rotation_speed = 4.25
'''
this is basically a reskin of orbit ball with parameters tuned so that it pushes the ball instead of orbiting it
TODO: make it so we drop control of the ball before traveling 1m from where we last took control of the ball to
abide by rules
'''
class dribble_ball(action):
  # reskin of orbit_ball, offset is 90 (ball radius is 120)
  # makes the robot push into the ball
  # results in "dribbling" the ball to target_loc
  def __init__(self, target_loc = False, offset = 90):
    action.__init__(self)
    self.pid = move_to()
    self.moving_to = False
    self.target_loc = target_loc
    self.spiral_factor = .4
    self.push_speed_factor = 1
    self.chase_down_factor = 4
    self.speed_mod_factor = 7
    self.offset = offset
    self.push_offset = offset
    
  # proxy_rot is not used it is a hack to let get_open with ball work easily
  def set_target(self, target_loc, proxy_rot):
    self.target_loc = target_loc
    
  def add(self, robot, game):
    self.pid.add(robot, game)
    action.add(self, robot, game)
    
  def run(self):
    if np.linalg.norm(self.robot.loc - self.target_loc) < 200:
      self.offset = 120
    else:
      self.offset = self.push_offset
  
    # pull from current location in by spiral factor
    ball_extrapolation = self.game.ball.loc
    robot_vec = self.robot.loc - ball_extrapolation
    robot_vec_scaled = robot_vec * self.offset / np.linalg.norm(robot_vec)
    target_loc = robot_vec_scaled * (1 - self.spiral_factor) + self.spiral_factor * robot_vec
    
    # get desired global angle and use the difference between that and current angle to rotate "target_loc"
    # by a scaled amount
    target_vec = ball_extrapolation - self.target_loc
    current_angle = -math.atan2(robot_vec[1], robot_vec[0])
    target_angle = -math.atan2(target_vec[1], target_vec[0])
    rotation_angle = -min_angle(current_angle - target_angle) * (1 - self.spiral_factor) / 2
    orbit_vec = convert_local(target_loc, rotation_angle)
    
    # if the move_to location is too close to the robot push it out by an amount
    # this amout increases by push_speed_factor if we are pushing the ball
    # and by chase_down_factor if we are behind where we want to be on the ball
    # speed_mod_factor is a baseline constant pushout factor
    move_to = orbit_vec + ball_extrapolation
    speed_mod_vec = move_to - self.robot.loc
    on_angle_factor = np.clip(math.pi - 10*abs(rotation_angle), 0, math.pi)**2
    off_angle_factor = np.clip(-math.pi/20 + abs(rotation_angle), 0, math.pi)**2
    if np.linalg.norm(speed_mod_vec) < 200:
      move_to = move_to + speed_mod_vec * (self.speed_mod_factor + self.push_speed_factor * on_angle_factor
                + off_angle_factor * self.chase_down_factor)
    
    # look at the ball 
    # call pid
    point_dir = (ball_extrapolation) - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    self.pid.set_target(move_to, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.moving_to = move_to
    return actions
