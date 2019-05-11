import numpy as np
import math
import time
import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/..') 
from pygame_simulator.PySim_noise import *
    

sys.path.insert(0, dirname)
from neutral_strategy import *
from offensive_strategy import *
from defensive_strategy import *
from passing_strategy import *
from state_machine import *
from strategy_helpers import *
    
class strategy:
  def __init__(self, game, is_blue):
    self.team = team(game, is_blue)
    self.neutral_id = 0
    self.offensive_id = 1
    self.defensive_id = 2
    self.passing_id = 3
    self.neutral = neutral_strategy(self.neutral_id, self.offensive_id, self.offensive_id, self.team)
    self.offense = offensive_strategy(self.offensive_id, self.neutral_id, self.defensive_id, self.passing_id, self.team)
    self.defense = defensive_strategy(self.defensive_id, self.neutral_id, self.offensive_id, self.team)
    self.passing = passing_strategy(self.passing_id, self.neutral_id, self.offensive_id, self.offensive_id, self.team)
    self.state_machine = state_machine([self.neutral, self.offense, self.defense, self.passing])
  def update(self):
    self.team.game.add_action(self.team.goalie_action, self.team.goalie.id, self.team.is_blue)
    self.state_machine.run()
      
  def reset(self):
    self.state_machine.reset()
      
if __name__ == "__main__":
  game = PYsim(6)
  
  blue_strategy = strategy(game, is_blue = True)
  yellow_strategy = strategy(game , is_blue = False)
  
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  
  select_id = 0
  select_blue = True
  while 1:
  
    '''select teams with Y key (yellow)
                         B key (blue)
       select robots with number keys (0-5)
       left click to place selected robot
    '''
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
      if event.type == KEYDOWN or event.type == KEYUP:
        keys = pygame.key.get_pressed()
        #press r-key to reset
        if keys[K_r]:
          game.reset()
          yellow_strategy.reset()
        if keys[K_1]:
          select_id = 1
        if keys[K_2]:
          select_id = 2
        if keys[K_3]:
          select_id = 3
        if keys[K_4]:
          select_id = 4
        if keys[K_5]:
          select_id = 5
        if keys[K_0]:
          select_id = 0
          
        if keys[K_b]:
          select_blue = True
        if keys[K_y]:
          select_blue = False
      if event.type == MOUSEBUTTONDOWN or event.type == MOUSEMOTION:
        pressed1, pressed2, pressed3 = pygame.mouse.get_pressed()
        #left mouse button
        if pressed1:
          if select_blue:
            game.blue_robots_internal[select_id].loc = game.convert_to_field_position(pygame.mouse.get_pos())
          else:
            game.yellow_robots_internal[select_id].loc = game.convert_to_field_position(pygame.mouse.get_pos())
          
    _, _, done = game.step(key_points = yellow_strategy.team.prints)
    
    #reset strategy when game ends
    if done:
      blue_strategy.reset()
      yellow_strategy.reset()
    
    
    #my offensive_strategy currently isn't good enough to deal with my defensive_strategy
    #blue_strategy.update()
    
    
    yellow_strategy.update()