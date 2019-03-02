import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname + "/../..")
from basic_skills.src.basic_skills.move_to import *
from pysim.src.pysim.PySim import *

import threading
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons


def PID_tuning_tool(move_action):
  plt.figure(2)
  Pax = plt.axes([0.25, 0.1, 0.65, 0.03])
  Psl = Slider(Pax, "translation - P", 10, 50, valinit=move_action.locP)

  Dax = plt.axes([0.25, 0.15, 0.65, 0.03])
  Dsl = Slider(Dax, "translation - D", 7, 40, valinit=move_action.locD)
  
  RPax = plt.axes([0.25, 0.2, 0.65, 0.03])
  RPsl = Slider(RPax, "rotational - P", 40, 200, valinit=move_action.rotP)

  RDax = plt.axes([0.25, 0.25, 0.65, 0.03])
  RDsl = Slider(RDax, "rotational - D", 40, 200, valinit=move_action.rotD)
  
  TSax = plt.axes([0.25, 0.3, 0.65, 0.03])
  TSsl = Slider(TSax, "translational speed", 50, 250, valinit=move_action.translational_control_speed)

  RSax = plt.axes([0.25, 0.35, 0.65, 0.03])
  RSsl = Slider(RSax, "rotational speed", 40, 100, valinit=move_action.rot_control_speed)

  def update(event):
    plt.figure(1)
    move_action.translational_control_speed = TSsl.val
    move_action.locP = Psl.val
    move_action.locD = Dsl.val

    move_action.rot_control_speed = RSsl.val
    move_action.rotP = RPsl.val
    move_action.rotD = RDsl.val
    event.canvas.draw()
    plt.clf()
  plt.gcf().canvas.mpl_connect('button_press_event', update)
  plt.show()

if __name__ == "__main__":
  max_bots_per_team = 6
  game = PYsim(max_bots_per_team)
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  move_action = MoveTo(game)
  move_action.set_target(np.array([0,0]), 0)
  '''
  make a separate thread to tune PID
  matplotlib doesn't play nice in the same thread
  '''
  tool_thread = threading.Thread(target = PID_tuning_tool, 
                  args = (move_action,))
  tool_thread.start()
  
  game.add_action(move_action, 0, True)
  j = 0
  while 1:
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
      if event.type == MOUSEBUTTONDOWN:
        pressed1, pressed2, pressed3 = pygame.mouse.get_pressed()
        
        #left mouse button
        if pressed1:
          print("high")
          game.blue_robots_internal[0]._action.set_target(game.convert_to_field_position(pygame.mouse.get_pos()), 0)
    new_time = clock.tick()
    
    '''
    put the robot in a random location and have it move back to center
    '''
    # if move_action.done():
      # random_location = np.random.uniform(-1, 1, size = [2])*np.array([3000, 2500])
      # random_velocity = np.random.uniform(-1, 1, size = [2])*np.array([500, 500])
      # random_rotation = np.random.uniform(-2*math.pi, 2*math.pi)
      # game.blue_robots_internal[0].loc = random_location
      # game.blue_robots_internal[0].rot = random_rotation
      # game.blue_robots_internal[0].velocity = random_velocity
      # move_action.set_target(random_location, random_rotation)
      
    game.step()
    j += 1
    ttime = new_time
