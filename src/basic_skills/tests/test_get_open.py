import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/../..')
from basic_skills.source.get_open import *
from pygame_simulator.PySim_noise import *

if __name__ == "__main__":
  max_bots_per_team = 6
  game = PYsim(max_bots_per_team)
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  
  # '''
  # Create ball controller
  # assumes yellow_robot 1 has the highest pass values and yellow_robot 0 has the ball or can easily get the ball
  # '''
  game.add_action(get_open_with_ball([game.yellow_robots[1]], game.blue_robots, game.yellow_robots), 0, False)
  
  '''
  Create the strikers
  assumes yellow_robot 0 has the ball and [-5000,0] is the goal
  '''
  
  blocker_enemies = [robot(True, -1, None), robot(True, -1, None)]
  their_goal = np.array([-5000,0])
  blocker_enemies[0].loc = their_goal + np.array([0, game.goal_height/2])
  blocker_enemies[1].loc = their_goal + np.array([0, -game.goal_height/2])
  for i in game.yellow_robots[1:3]:
    game.add_action(striker(game.yellow_robots[0], np.array([-5000,0]), game.blue_robots + blocker_enemies, game.yellow_robots), i.id, False)
      
  '''
  Create the fielders
  assumes yellow_robot 0 has the ball and yellow_robot i.id - 2 is a striker
  '''
  for i in game.yellow_robots[3:5]:
    game.add_action(fielder(game.yellow_robots[0], game.yellow_robots[i.id - 2], game.blue_robots + blocker_enemies, game.yellow_robots), i.id, False)
    
    
  while 1:
      
      
    '''
    handle resets and allow robots to be placed for debugging
    '''
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
      if event.type == MOUSEBUTTONDOWN:
        pressed1, pressed2, pressed3 = pygame.mouse.get_pressed()
        
        # left mouse button
        if pressed1:
          print("high")
          game.yellow_robots_internal[1].loc = game.convert_to_field_position(pygame.mouse.get_pos())
          
        # right mouse button
        if pressed3:
          print("hiiii")
          game.yellow_robots_internal[0].loc = game.convert_to_field_position(pygame.mouse.get_pos())
      if event.type == KEYDOWN or event.type == KEYUP:
        keys = pygame.key.get_pressed()
        
        # press r-key to reset
        if keys[K_r]:
          game.reset()
    new_time = clock.tick()
    game.step(key_points = game.yellow_robots[1].action.prints)
    ttime = new_time