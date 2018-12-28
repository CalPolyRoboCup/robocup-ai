import sys
sys.path.insert(0, '../../GR_sim_networking')


from GR_Interact import *
from move_to import *

if __name__ == "__main__":
  max_bots_per_team = 6
  game = GRsim(max_bots_per_team)
  move_action = move_to()
  game.blue_robots[0].add_action(move_action)
  for j in range(10000):
    if j % 300 == 0:
      random_location = np.random.uniform(-1, 1, size = [2])*np.array([2000, 1500])
      random_rotation = np.random.uniform(-2*math.pi, 2*math.pi)
      move_action.set_target(random_location, random_rotation)#np.array([0,0]), random_rotation)#
    game.step()
