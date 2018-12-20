import numpy as np
import sys
sys.path.insert(0, '../../GR_sim_networking')
sys.path.insert(0, '/home/adleywong/Repositories/robocup-ai/src/move_to')

from GR_Interact import *
from dribble_ball import *

def reset(game):
  print("update")
  game.sync_with_sim()

if __name__ == "__main__":
  max_bots_per_team = 6
  game = GRsim(max_bots_per_team)
  move_action = move_to()
  reset(game)
  game.blue_robots[0].add_action(move_action)
  print(game.ball.loc)
  for j in range(10000):
    if j % 300 == 0:
      move_action.set_target(np.array([1000, 0]), 0)
    game.step()