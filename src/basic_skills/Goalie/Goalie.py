import numpy as np
import sys
sys.path.insert(0, '../..')
from basic_skills.action import *
from basic_skills.move_to.move_to import move_to
from pygame_simulator.PySim_noise import *

 


class Goalie(action):
  # Covers a pass from target_ball to target_loc
  # for goalie
  # identical to cover with "lead_target_robot" False
  def __init__(self):
    action.__init__(self)
    self.goal = np.array([0,0])
    self.pid = move_to()
    self.orbit = 1500
    
  def add(self, robot, game):
    action.add(self, robot, game)
    move_to.add(self.pid, robot, game)
    if self.robot.is_blue:
      self.goal = np.array([-6000,0])
    else:
      self.goal = np.array([6000,0])
      
  def push_out_function(self, dist):
    '''
    brief: we want to move forward to intercept the ball and cover more of the goal.
            This bit of math squashes the distance from the ball to the goal so that 
            if the ball is close to the goal a large number is returned. Otherwise a
    params: dist - distance from ball to goal 
    returns: value - squashed value as described above.
    '''
    #small number.
    return 1500000/(100 + dist)
  
  def run(self):
  
    adjustment = self.push_out_function(np.linalg.norm(self.game.ball.loc - self.goal))
    
    #recoil to buy time and absorb shock if the ball is free
    # if self.game.ball.controler == False:
      # adjustment = -adjustment
      
    #we will position ourselves between the ball and the goal 
    push_out_distance = self.orbit + adjustment
    push_vec = self.game.ball.loc - self.goal
    
    #print(adjustment, -np.linalg.norm(push_vec))
    if push_out_distance > np.linalg.norm(push_vec):
      push_out_distance = np.linalg.norm(push_vec) - 40
    move_to = self.goal + push_vec * push_out_distance / np.linalg.norm(push_vec)
    point_dir = self.game.ball.loc - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    
    #bypass movement restrictions and set target location directly
    self.pid.target_loc = move_to
    self.pid.target_rot = target_rot
    actions = self.pid.run()
    self.actions = actions
    self.move_to = move_to
    return actions
    
   