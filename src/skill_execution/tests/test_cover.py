import sys

from pysim.PySim_noise import *
from skill_execution.cover import *


game = PYsim(6)

cover_action = cover(np.array([0,3000]), game.yellow_robots[0], 1, min_interpose_offset = 50)
CR_action = cover_robots(game.blue_robots[0], game.yellow_robots[0])
move_action = move_to()

game.add_action(cover_action, 0, True)
game.add_action(move_action, 0, False)
game.add_action(CR_action, 1, False)

target_locs = [[-1000, 1000], [1000, -1000]]
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
  new_time = clock.tick()
  if game.yellow_robots[0].action.done():
    if i == 0:
      i = 1
    else:
      i = 0
    move_action.set_target(target_locs[i], 0)
  game.step()
  ttime = new_time
