import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '../..')
from basic_skills.action import *
from basic_skills.move_to.move_to import *
from basic_skills.helper_functions import *


class cover(action):
  robot_rotation_speed = 4.25
  EPS = 0.001
  '''
  covers a pass from target_robot to target_loc
  '''
  def __init__(self, target_loc, target_robot, interpose_factor = .5, min_interpose_offset = 0, interpose_weight = 25, lead_target_robot = True):
    action.__init__(self)
    self.pid = move_to()
    self.move_to = False
    self.target_loc = target_loc
    self.target_robot = target_robot
    
    #whether to take the target_robot's rotation into account 
    self.lead_target_robot = lead_target_robot
    
    # factor from 0 to 1 describing how the robot will position itself between the
    # target_loc and target_robot
    # 1 is on target_loc
    # 0 is on target_robot
    self.interpose_factor = interpose_factor
    
    # minimum distance allowed between target_loc and robot
    # useful for body blocking
    self.min_interpose_offset = min_interpose_offset
    
    # controls how quickly the robot switches from quickest path to target loc movement
    self.interpose_weight = interpose_weight
    
  def add(self, robot, game):
    self.robot = robot
    self.pid.robot = robot
    action.add(self, robot, game)
    
  def run(self):
    '''
    move to a position interpose_factor between the target robot position and the target location
    If self.lead_target_robot is True we will interpolate the target robot position based on the time it would 
      take to rotate to face the target_loc
      
    On our way to the point we also average the final location (calculated above) and the point on the line closest
    to the robot. When we are farther from the line the point on the line closest to the robot is given a larger 
    weight so that we get to the ball faster
    '''
    
    #get the position the ball will be shot from
    shooting_pos = self.target_robot.loc
    if self.lead_target_robot:
      shooting_vec = self.target_loc - self.target_robot.loc
      delta_angle = abs(min_angle(self.target_robot.rot - math.atan2(shooting_vec[1], shooting_vec[0])))
      shooting_pos = shooting_pos + self.target_robot.velocity * delta_angle / cover.robot_rotation_speed
      
    #Find the point that will put you between the shooting position and the ball the quickest
    quickest_loc = drop_perpendicular(self.robot.loc, shooting_pos, shooting_vec)
    
    #if we are too close to the ball force a offset
    if np.linalg.norm(shooting_vec) * self.interpose_factor < self.min_interpose_offset and np.linalg.norm(shooting_vec) > cover.EPS:
      interpose_loc = shooting_pos + shooting_vec * self.min_interpose_offset / np.linalg.norm(shooting_vec)
      
    #otherwise get the point interpose_factor between the shooting position and the ball
    else:
      interpose_loc = shooting_pos + shooting_vec * self.interpose_factor
      
    #use our distance to the ball to average the two points (interpose_loc, quickest_loc) together
    quickest_distance = np.linalg.norm(quickest_loc - self.robot.loc)
    move_to = ((quickest_loc * quickest_distance + interpose_loc * self.interpose_weight)
                  / (self.interpose_weight + quickest_distance))
    
    #look at the ball
    point_dir = self.target_robot.loc - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    
    #call PID
    self.pid.set_target(move_to, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.move_to = move_to
    return actions
    
class cover_robots(action):
  robot_rotation_speed = 4.25
  EPS = 0.001
  '''
  Covers a pass from target_bot1 to target_bot2
  '''
  def __init__(self, target_bot1, target_bot2, interpose_factor = .5, min_interpose_offset = 190, interpose_weight = 25, lead_target_robot = True):
    action.__init__(self)
    self.pid = move_to()
    self.move_to = False
    self.target_bot1 = target_bot1
    self.target_bot2 = target_bot2
    self.lead_target_robot = lead_target_robot
    
    # factor from 0 to 1 describing how the robot will position itself between the
    # target_bot1 and target_bot2
    # 1 is on target_bot1
    # 0 is on target_bot2
    self.interpose_factor = interpose_factor
    
    # minimum distance allowed between target_bot1 and robot
    # useful for body blocking
    self.min_interpose_offset = min_interpose_offset
    
    # controls how quickly the robot switches from quickest path to target loc movement
    self.interpose_weight = interpose_weight
    
  def add(self, robot, game):
    self.robot = robot
    self.pid.robot = robot
    action.add(self, robot, game)
    
  def run(self):
    '''
    move to a position interpose_factor between the target robot position and the target location
    If self.lead_target_robot is True we will interpolate the target robot position based on the time it would 
      take to rotate to face the target_loc
      
    On our way to the point we also average the final location (calculated above) and the point on the line closest
    to the robot. When we are farther from the line the point on the line closest to the robot is given a larger 
    weight so that we get to the ball faster
    '''
    
    #get the position the ball will be shot from and too
    shooting_pos = self.target_bot2.loc
    target_loc = self.target_bot1.loc
    shooting_vec = self.target_bot1.loc - self.target_bot2.loc
    if self.lead_target_robot:
      if type(self.target_bot1) is robot:
        delta_angle = abs(min_angle(self.target_bot1.rot - math.atan2(shooting_vec[1], shooting_vec[0])))
        target_loc = target_loc + self.target_bot1.velocity * delta_angle / cover_robots.robot_rotation_speed
      
      if type(self.target_bot2) is robot:
        delta_angle = abs(min_angle(self.target_bot2.rot - math.atan2(shooting_vec[1], shooting_vec[0])))
        shooting_pos = shooting_pos + self.target_bot2.velocity * delta_angle / cover_robots.robot_rotation_speed
      
    #Find the point that will put you between the shooting position and the ball the quickest
    quickest_loc = drop_perpendicular(self.robot.loc, shooting_pos, shooting_vec)
    
    #if we are too close to the ball force a offset
    if np.linalg.norm(shooting_vec) * self.interpose_factor < self.min_interpose_offset and np.linalg.norm(shooting_vec) > cover_robots.EPS:
      interpose_loc = shooting_pos + shooting_vec * self.min_interpose_offset / np.linalg.norm(shooting_vec)
      
    #otherwise get the point interpose_factor between the shooting position and the ball
    else:
      interpose_loc = shooting_pos + shooting_vec * self.interpose_factor
      
    #use our distance to the ball to average the two points (interpose_loc, quickest_loc) together
    quickest_distance = np.linalg.norm(quickest_loc - self.robot.loc)
    move_to = ((quickest_loc * quickest_distance + interpose_loc * self.interpose_weight)
                  / (self.interpose_weight + quickest_distance))
    
    #look at the ball
    point_dir = self.target_bot1.loc - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    
    #call PID
    self.pid.set_target(move_to, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.move_to = move_to
    return actions