import sys
sys.path.insert(0, '/Users/nathan/Documents/robocup-ai/src')
#sys.path.insert(0, '../../GR_sim_networking')

from pygame_simulator.PySim import *
from basic_skills.move_to.move_to import *

import threading
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons


def PID_tuning_tool(move_action):
  plt.figure(2)
  Pax = plt.axes([0.25, 0.1, 0.65, 0.03])
  Psl = Slider(Pax, "translation - P", 10, 40, valinit=move_action.locP)

  Dax = plt.axes([0.25, 0.15, 0.65, 0.03])
  Dsl = Slider(Dax, "translation - D", 7, 30, valinit=move_action.locD)
  
  RPax = plt.axes([0.25, 0.2, 0.65, 0.03])
  RPsl = Slider(RPax, "rotational - P", 0, 50, valinit=move_action.rotP)

  RDax = plt.axes([0.25, 0.25, 0.65, 0.03])
  RDsl = Slider(RDax, "rotational - D", 0, 50, valinit=move_action.rotD)
  
  TSax = plt.axes([0.25, 0.3, 0.65, 0.03])
  TSsl = Slider(TSax, "translational speed", 50, 250, valinit=move_action.translational_control_speed)

  RSax = plt.axes([0.25, 0.35, 0.65, 0.03])
  RSsl = Slider(RSax, "rotational speed", 0, 50, valinit=move_action.rot_control_speed)

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
    #print("tool_thread", move_action.locP)
  plt.gcf().canvas.mpl_connect('button_press_event', update)
  plt.show()

if __name__ == "__main__":
  max_bots_per_team = 6
  game = PYsim(max_bots_per_team)
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  move_action = move_to()
  move_action.set_target(np.array([0,0]), 0)
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
      if event.type == KEYDOWN or event.type == KEYUP:
        keys = pygame.key.get_pressed()
        key_action.keypress_update(keys)
    new_time = clock.tick()
    if j % 300 == 0:
      random_location = np.random.uniform(-1, 1, size = [2])*np.array([3000, 2500])
      random_velocity = np.random.uniform(-1, 1, size = [2])*np.array([500, 500])
      random_rotation = np.random.uniform(-2*math.pi, 2*math.pi)
      game.blue_robots[0].loc = random_location
      game.blue_robots[0].rot = random_rotation
      game.blue_robots[0].velocity = random_velocity
      #print("game thread", move_action.locP)
      #move_action.set_target(random_location, random_rotation)#np.array([0,0]), random_rotation)#
      #print(random_location)
    # if j % 500 == 150:
      # print(move_action.action)
    game.step()
    j += 1
    ttime = new_time
