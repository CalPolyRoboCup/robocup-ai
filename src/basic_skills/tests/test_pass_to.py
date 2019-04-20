import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/../..')
from basic_skills.source.pass_to import *
from basic_skills.source.Ball_Interception import *
from pygame_simulator.PySim_noise import *

if __name__ == "__main__":
  game = PYsim(6)
  pass_action = pass_to(game.yellow_robots[0])
  move_action = move_to()
  
  extern_passing = False
  
  intercept_action = intercept_ball()
  game.add_action(pass_action, 0, True)
  game.add_action(move_action, 0, False)
  target_locs = [[-1000, 2000], [-1000, -2000], [1000, -2000], [1000,2000]]
  move_action.set_target(target_locs[0], 0)
  i = 0
  
  passing_robot = game.blue_robots_internal[0]
  moving_robot = game.yellow_robots_internal[0]
  passing_robot_is_blue = True
  moving_robot_is_blue = False
  
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
          moving_robot.loc = game.convert_to_field_position(pygame.mouse.get_pos())
          
        # right mouse button
        if pressed2:
          print("low")
          passing_robot.loc = game.convert_to_field_position(pygame.mouse.get_pos())
          
        
    new_time = clock.tick()
    if moving_robot.action.done():
      i = (i + 1) % len(target_locs)
      move_action.set_target(target_locs[i], 0)
      
    if pass_action.done():
      
      i = (i + 1) % len(target_locs)
      move_action.set_target(target_locs[i], 0)
      
      game.add_action(move_action, 0, passing_robot_is_blue)
      game.add_action(intercept_action, 0, not passing_robot_is_blue)
      extern_passing = True
      robot_buf = passing_robot
      passing_robot = moving_robot
      moving_robot = robot_buf
      passing_robot_is_blue = not passing_robot_is_blue
      moving_robot_is_blue = not moving_robot
      print("passed")
      
    if extern_passing and game.ball.controler != False and game.ball.controler.is_blue == passing_robot_is_blue:
      game.add_action(pass_action, 0, passing_robot_is_blue)
      pass_action.target_robot = moving_robot
      print("recieved", passing_robot_is_blue)
      extern_passing = False
      
      
    game.step()
    ttime = new_time