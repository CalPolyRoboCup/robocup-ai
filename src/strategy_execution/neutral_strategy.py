import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from state_machine import state

import numpy as np
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/..')
from basic_skills.source.get_open import *
from basic_skills.source.Ball_Interception import *
from strategy_helpers import get_closest, assign_strikers_and_fielders

#assigns actions to robots when neither team controls the ball
class neutral_strategy(state):
  def __init__(self, id, offensive_id, defensive_id, team):
    state.__init__(self, id)
    self.offensive_id = offensive_id
    self.defensive_id = defensive_id
    self.team = team
  def setup(self, closest = None):
    if closest == None:
      closest = get_closest(self.team.game.ball, self.team.allies)
    self.team.ball_controler = closest.id
    #print("neutral")
    free_allies = [a for a in self.team.allies]
    
    #the robot closest to the ball tries to get the ball
    self.team.game.add_action(self.team.intercept_action, closest.id, self.team.is_blue)
    for fa in free_allies:
      if fa.id == closest.id:
        free_allies.remove(fa)
        break
        
    #other robots spread out
    assign_strikers_and_fielders(self.team, free_allies)
  def update(self):
    closest = get_closest(self.team.game.ball, self.team.allies)
    if closest.id != self.team.ball_controler:
      self.setup(closest)
    
    #if someone has gotten the ball
    if self.team.game.ball.controler != False:
      self.team.ball_controler = self.team.game.ball.controler.id
      if self.team.game.ball.controler.is_blue == self.team.is_blue:
        return self.offensive_id
      else:
        return self.defensive_id
    return self.id