import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname + "/../..")
from basic_skills.src.basic_skills.orbit_ball import *
from pysim.src.pysim.PySim import *

'''
yellow robot moves back and forth while the blue robot orbits the ball to look at the yellow robot
'''
if __name__ == "__main__":
  game = PYsim(6)
  orbit_action = OrbitBall(game, np.array([0,3000]))
  move_action = MoveTo(game)
  game.add_action(orbit_action, 0, True)
  game.add_action(move_action, 0, False)
  print(game.blue_robots[0]._action, game.yellow_robots[0]._action)
  target_locs = [[-1000, 2000], [1000, -1000]]
  move_action.set_target(target_locs[0], 0)
  i = 0
  
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  while 1:
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
    
    game.blue_robots[0]._action.target_loc = game.yellow_robots[0].loc + game.yellow_robots[0].velocity
        
    new_time = clock.tick()
    if game.yellow_robots[0]._action.done():
      if i == 0:
        i = 1
      else:
        i = 0
      move_action.set_target(target_locs[i], 0)
    game.step()
    ttime = new_time