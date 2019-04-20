import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/../..')
from basic_skills.source.Ball_Interception import *
from basic_skills.tests.Catch_Pygym import *

if __name__ == "__main__":
  max_bots_per_team = 6
  game = Ball_Intercept_PYGym(max_bots_per_team)
  intercept_action = intercept_ball()
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