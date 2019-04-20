import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/../..')
from basic_skills.source.orbit_ball import *
from pygame_simulator.PySim_noise import *

if __name__ == "__main__":
  game = PYsim(6)
  orbit_action = orbit_ball(np.array([0,3000]))
  move_action = move_to()
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
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
        
      if event.type == MOUSEBUTTONDOWN:
        pressed1, pressed2, pressed3 = pygame.mouse.get_pressed()
        
        # left mouse button
        if pressed1:
          print("high")
          game.yellow_robots_internal[0].loc = game.convert_to_field_position(pygame.mouse.get_pos())
          
    
    game.blue_robots[0].action.target_loc = game.yellow_robots[0].loc + game.yellow_robots[0].velocity
        
    new_time = clock.tick()
    if game.yellow_robots[0].action.done():
      if i == 0:
        i = 1
      else:
        i = 0
      move_action.set_target(target_locs[i], 0)
    game.step()
    ttime = new_time