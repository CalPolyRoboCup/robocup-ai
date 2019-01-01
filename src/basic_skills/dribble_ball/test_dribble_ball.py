import numpy as np
import sys
sys.path.insert(0, '../../../src')

from pygame_simulator.PySim import *
from basic_skills.ball_interception.Ball_Interception import *
from basic_skills.ball_interception.Catch_Pygym import *
from basic_skills.move_to.move_to import *

if __name__ == "__main__":
  max_bots_per_team = 6
  game = Ball_Intercept_PYGym(max_bots_per_team)
  intercept_action = intercept_ball()
  move_action = move_to()
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  game.add_action(intercept_action, 0, False)
  while 1:
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
    new_time = clock.tick()
    game.step()
    ttime = new_time