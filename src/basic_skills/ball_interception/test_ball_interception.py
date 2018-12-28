import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/Users/wulfkine/repos/robocup-ai/src')
from pygame_simulator.PySim import *
from basic_skills.ball_interception.Ball_Interception import *
from basic_skills.ball_interception.Catch_Pygym import *

if __name__ == "__main__":
  max_bots_per_team = 6
  game = Ball_Intercept_PYGym(max_bots_per_team)
  intercept_action = intercept_ball()
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  game.yellow_robots[0].add_action(intercept_action)
  while 1:
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
    new_time = clock.tick()
    game.step()
    ttime = new_time
