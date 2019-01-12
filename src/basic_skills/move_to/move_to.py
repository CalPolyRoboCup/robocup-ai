import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '../../../src')
from basic_skills.action import *
from basic_skills.helper_functions import *

from pygame_simulator.PySim import *
import threading
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons

import numpy as np

class move_to(action):
  def __init__(self, target_loc = False, target_rot = False):
    action.__init__(self)
    self.target_loc = target_loc
    self.target_rot = target_rot

    self.translational_control_speed = 150
    self.locP = 34.3#.0045
    self.locD = 15#.00075
    self.pivot_center = np.array([-.1,0])
    self.pivot_factor = 0#.0004

    self.rot_control_speed = 100
    self.rotP = 100
    self.rotD = 40
  def set_target(self, target_loc, target_rot):
    '''
    make target loc valid and 
    set self.target_loc 
    and self.target_rot
    '''
    if target_loc[0] < -5500:
      target_loc[0] = -5500
    if target_loc[0] > 5500:
      target_loc[0] = 5500
    if target_loc[1] < -3500:
      target_loc[1] = -3500
    if target_loc[1] > 3500:
      target_loc[1] = 3500
    self.target_loc = out_of_endzone(target_loc)
    self.target_rot = target_rot
  def run(self):
    norm_vel, tang_vel = self.PID_loc()
    rot_vel = self.PID_rot()
    action = [0,0,norm_vel, tang_vel, rot_vel]
    self.action = action
    #print("move to P - ", self.locP)
    return action
  def PID_loc(self):
    vector = self.target_loc - self.robot.loc
    distance = np.linalg.norm(vector)
    local = convert_local(vector, -self.robot.rot)
    
    local_vel = convert_local(self.robot.velocity, -self.robot.rot)
    loc_PID = self.locP*local - self.locD*local_vel
    #print(self.locP*local, -local_vel*self.locD)
    pidMag = np.linalg.norm(loc_PID)
    if pidMag > self.translational_control_speed:
      loc_PID *= self.translational_control_speed/pidMag

    #correct for induced twist
    #pivot_vel = local_vel - self.pivot_center
    #cross_prod = pivot_vel[0] * loc_PID[1] - pivot_vel[1] * loc_PID[0]
    #rot_vel = cross_prod * self.pivot_factor
    #print(rot_vel, self.locP*local, -self.locD*local_vel)
    return loc_PID[0], loc_PID[1]#, rot_vel
  def PID_rot(self):
    distance = min_angle(self.target_rot - self.robot.rot)
    PID = self.rotP*distance - self.rotD*self.robot.rot_vel
    if (PID > self.rot_control_speed):
      PID = self.rot_control_speed
    elif(PID < -self.rot_control_speed):
      PID = -self.rot_control_speed

    return PID
  def done(self):
    epsilon = 15
    #print(np.linalg.norm(self.target_loc - self.robot.loc), np.linalg.norm(self.robot.velocity))
    if np.linalg.norm(self.target_loc - self.robot.loc) < epsilon and np.linalg.norm(self.robot.velocity) < epsilon:
      return True
    return False

    
def PID_tuning_tool(move_action):
  plt.figure(2)
  Pax = plt.axes([0.25, 0.1, 0.65, 0.03])
  Psl = Slider(Pax, "translation - P", 10, 40, valinit=move_action.locP)

  Dax = plt.axes([0.25, 0.15, 0.65, 0.03])
  Dsl = Slider(Dax, "translation - D", 7, 30, valinit=move_action.locD)
  
  RPax = plt.axes([0.25, 0.2, 0.65, 0.03])
  RPsl = Slider(RPax, "rotational - P", 40, 100, valinit=move_action.rotP)

  RDax = plt.axes([0.25, 0.25, 0.65, 0.03])
  RDsl = Slider(RDax, "rotational - D", 40, 100, valinit=move_action.rotD)
  
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
      if event.type == KEYDOWN or event.type == KEYUP:
        keys = pygame.key.get_pressed()
        key_action.keypress_update(keys)
    new_time = clock.tick()
    
    '''
    put the robot in a random location and have it move back to center
    '''
    if j % 300 == 0:
      random_location = np.random.uniform(-1, 1, size = [2])*np.array([3000, 2500])
      random_velocity = np.random.uniform(-1, 1, size = [2])*np.array([500, 500])
      random_rotation = -math.pi#np.random.uniform(-2*math.pi, 2*math.pi)
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
