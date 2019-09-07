import sys
import os
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import threading
import pygame

dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/../..')
from basic_skills.source.PassTo import PassTo
from basic_skills.source.InterceptBall import InterceptBall
from basic_skills.source.MoveTo import MoveTo
from basic_skills.source.helper_functions import dist, angle_to
from pygame_simulator.PySim_noise import PYsim

PASS_DISTANCE = 3000

def distance_tune(target_locs):
  plt.figure(2)
  Pax = plt.axes([0.25, 0.1, 0.65, 0.03])
  Psl = Slider(Pax, "translation - P", 100, PASS_DISTANCE, valinit=PASS_DISTANCE)

  def update(event):
    plt.figure(1)
    target_locs.target_locs = [[-Psl.val, Psl.val], [-Psl.val, -Psl.val], [Psl.val, -Psl.val], [Psl.val, Psl.val]]
    event.canvas.draw()
    plt.clf()
  plt.gcf().canvas.mpl_connect('button_press_event', update)
  plt.show()

class tl_wrap():
  def __init__(self):
    self.target_locs = [[-PASS_DISTANCE, PASS_DISTANCE], [-PASS_DISTANCE, -PASS_DISTANCE], [PASS_DISTANCE, -PASS_DISTANCE], [PASS_DISTANCE,PASS_DISTANCE]]
    

if __name__ == "__main__":
  game = PYsim(6)
  pass_action = PassTo(game.yellow_robots[0])
  move_action = MoveTo()
  
  extern_passing = False
  
  intercept_action = InterceptBall()
  game.add_action(pass_action, 0, True)
  game.add_action(move_action, 0, False)
  tlw = tl_wrap()
  
  # open a window to tune the spacing of the robots
  # The UI is an infinite loop. I can't get the thread
  # to join cleanly.
  # tool_thread = threading.Thread(target = distance_tune, 
  #                 args = (tlw,))
  # tool_thread.start()
  
  move_action.set_target(tlw.target_locs[0], 0)
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
      if event.type == pygame.QUIT:
        kill = True
        pygame.quit()
        #tool_thread.join()
        sys.exit()
        
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
    if dist(moving_robot.loc, move_action.target_loc) < 100:
      i = (i + 1) % len(tlw.target_locs)
      move_action.set_target(tlw.target_locs[i], 0)
      
    if pass_action.done():
      
      i = (i + 1) % len(tlw.target_locs)
      move_action.set_target(tlw.target_locs[i], 0)
      
      game.add_action(move_action, 0, passing_robot_is_blue)
      game.add_action(intercept_action, 0, not passing_robot_is_blue)
      extern_passing = True
      robot_buf = passing_robot
      passing_robot = moving_robot
      moving_robot = robot_buf
      passing_robot_is_blue = not passing_robot_is_blue
      moving_robot_is_blue = not moving_robot
      print("passed")
      capture_ticks = 30
      
    if extern_passing and game.ball.controler != False and game.ball.controler.is_blue == passing_robot_is_blue:
      capture_ticks -= 1
      if capture_ticks == 0:
        game.add_action(pass_action, 0, passing_robot_is_blue)
        pass_action.target_robot = moving_robot
        print("recieved", passing_robot_is_blue)
        extern_passing = False
      
    moving_robot.action.target_rot = angle_to(passing_robot.loc, moving_robot.loc)

    game.step(key_points = [intercept_action.target_loc])
    ttime = new_time