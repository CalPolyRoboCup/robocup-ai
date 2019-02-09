import numpy as np
import math
import sys
sys.path.insert(0, '..')
from pygame_simulator.PySim import *


def testing_environment(game):
  
  
  '''
  handle resets and allow robots to be placed for debugging
  '''
  for event in pygame.event.get():
    if event.type == QUIT:
      pygame.quit()
      sys.exit()
    if event.type == MOUSEBUTTONDOWN:
      pressed1, pressed2, pressed3 = pygame.mouse.get_pressed()
      
      #left mouse button
      if pressed1:
        game.yellow_robots_internal[1].loc = game.convert_to_field_position(pygame.mouse.get_pos())
        
      #right mouse button
      if pressed3:
        game.yellow_robots_internal[0].loc = game.convert_to_field_position(pygame.mouse.get_pos())
    if event.type == KEYDOWN or event.type == KEYUP:
      keys = pygame.key.get_pressed()
      
      #press r-key to reset
      if keys[K_r]:
        game.reset()
    new_time = clock.tick()
    game.step(key_points = game.yellow_robots[1].action.prints)
    ttime = new_time