import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from state_machine import state

dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/..')
from basic_skills.source.Ball_Interception import *
from basic_skills.source.pass_to import *

#assigns actions to roots while the ball is being passed by this team
class passing_strategy(state):
  def __init__(self, id, neutral_id, offensive_id, defensive_id, team):
    state.__init__(self, id)
    self.offensive_id = offensive_id
    self.neutral_id = neutral_id
    self.defensive_id = defensive_id
    self.team = team
    
    self.pass_timeout_time = 500
    self.pass_timeout = 0
  def setup(self):
    print("pass to", self.team.pass_action.target_robot.id)
    
    if self.team.pass_action.target_robot.id != -1:
      self.team.game.add_action(self.team.intercept_action,  self.team.pass_action.target_robot.id, self.team.is_blue)
      
    self.team.fielder_actions[2].target_to_support = self.team.pass_action.target_robot
    self.team.game.add_action(self.team.fielder_actions[2], self.team.ball_controler, self.team.is_blue)
    
    self.pass_timeout = self.pass_timeout_time
  def update(self):
    #on entering state assign the robot to receive pass to intercept_ball and the robot that just passed to fielder
      
    self.pass_timeout -= 1
      
    #state machine transitions
    if self.team.game.ball.controler != False:
      
      #if they have the ball under a robots control
      if self.team.game.ball.controler.is_blue != self.team.is_blue:
        print("interception")
        return self.defensive_id
        
      #if we have the ball under a robots control
      if self.team.game.ball.controler.is_blue == self.team.is_blue and self.team.game.ball.controler.id != self.team.ball_controler:
        self.team.ball_controler = self.team.game.ball.controler.id
        print("recieved", self.team.ball_controler)
        return self.offensive_id
        
    if self.pass_timeout == 0:
      return self.neutral_id
      
    return self.id