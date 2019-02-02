import sys
sys.path.insert(0, '/Users/nathan/Documents/robocup-ai/src')
from basic_skills.ball_interception.Ball_Interception2 import *
from basic_skills.ball_interception.Catch_Pygym import *

if __name__ == "__main__":
  max_bots_per_team = 6
  game = Ball_Intercept_PYGym(max_bots_per_team)
  intercept_action = intercept_ball()
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  game.add_action(intercept_action, 0, False)
    for event in pygame.event.get():
  while 1:
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
    new_time = clock.tick()
    game.step()
    ttime = new_time