import sys
import os
import pygame
import numpy as np
dirname = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, dirname+'/../..')
from basic_skills.source.OrbitBall import OrbitBall
from basic_skills.source.MoveTo import MoveTo
from pygame_simulator.PySim_noise import PYsim

if __name__ == "__main__":
  game = PYsim(6)
  orbit_action = OrbitBall(np.array([0,3000]))
  move_action = MoveTo()
  game.add_action(orbit_action, 0, True)
  game.add_action(move_action, 0, False)
  target_locs = [[-1000, 2000], [1000, -1000]]
  move_action.set_target(target_locs[0], 0)
  i = 0
  
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  while 1:
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        pygame.quit()
        sys.exit()
        
      if event.type == pygame.MOUSEBUTTONDOWN:
        pressed1, pressed2, pressed3 = pygame.mouse.get_pressed()
        mouse_pos = game.convert_to_field_position(pygame.mouse.get_pos())

        if pressed1:
          game.ball_internal.loc = mouse_pos
          game.ball_internal.velocity = np.array([0,0])
          
        if pressed3:
          orbit_action.set_target(mouse_pos)
        
    new_time = clock.tick()
    if game.yellow_robots[0].action.done():
      if i == 0:
        i = 1
      else:
        i = 0
      move_action.set_target(target_locs[i], 0)
    game.step()
    ttime = new_time