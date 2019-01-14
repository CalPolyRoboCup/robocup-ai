import sys
sys.path.insert(0, '../../GR_sim_networking')
import numpy as np
from math import *

from GR_Interact import *
from move_to import *
from kick import *
from ball_orbit import *

if __name__ == "__main__":
  max_bots_per_team = 6
  #Shoots the ball to the center of the field
  kick_to_loc = [0, 0]
  game = GRsim(max_bots_per_team)
  orbit_action = ball_orbit(np.array([0, 3000]))
  orbit_action.add(game.blue_robots[0], game)
  move_action = move_to()
  game.blue_robots[0].add_action(orbit_action)
  game.blue_robots[0].add_action(move_action)
  game.step()
  while (1):
    robot_dest = calc_kick_setup(kick_to_loc, game.ball.loc)
    robot_angle = calc_angle(kick_to_loc, game.ball.loc)
    move_action.set_target(robot_dest, robot_angle)
    game.step()
    print(game.ball.loc)
    if (abs(game.blue_robots[0].loc[0] - robot_dest[0]) < 1) & (abs(game.blue_robots[0].loc[1] - robot_dest[1]) < 1):
      
      orbit_action.run()
      kick_action = kick()
      game.blue_robots[0].add_action(kick_action)
      game.step()
      break
