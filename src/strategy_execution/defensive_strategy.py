import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from state_machine import state
from strategy_helpers import get_closest

import numpy as np
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/..')
from basic_skills.source.cover import *

#assigns actions to robots when the enemy has the ball
class defensive_strategy(state):
  def __init__(self, id, neutral_id, offensive_id, team):
    state.__init__(self, id)
    self.offensive_id = offensive_id
    self.neutral_id = neutral_id
    self.team = team
  def setup(self):
    #print("defend")
    free_allies = [a for a in self.team.allies]
    open_enemies = [e for e in self.team.enemies]
    
    closest = get_closest(self.team.enemies[self.team.ball_controler], free_allies)
    self.team.game.add_action(cover(self.team.my_goal, self.team.enemies[self.team.ball_controler], min_interpose_offset = 190), closest.id, self.team.is_blue)
    
    for oe in open_enemies:
      if self.team.enemies[self.team.ball_controler] == oe.id:
        open_enemies.remove(oe)
    for fa in free_allies:
      if closest.id == fa.id:
        free_allies.remove(fa)
    
    for e in open_enemies:
      if len(free_allies) == 0:
        break
      closest = get_closest(e, free_allies)
      self.team.game.add_action(cover_robots(e, self.team.enemies[self.team.ball_controler], min_interpose_offset = 190), closest.id, self.team.is_blue)
      for fa in free_allies:
        if closest.id == fa.id:
          free_allies.remove(fa)
          break
  def update(self):
    #on entering state assign each robot an enemy to guard
            
    #state machine transition 
    if self.team.game.ball.controler != False and self.team.game.ball.controler.is_blue == self.team.is_blue:
      self.team.ball_controler = self.team.game.ball.controler.id
      return self.offensive_id
      
    elif np.linalg.norm(self.team.enemies[self.team.ball_controler].loc - self.team.game.ball.loc) > 500:
      return self.neutral_id
      
    return self.id