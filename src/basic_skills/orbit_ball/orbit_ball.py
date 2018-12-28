import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/Users/nathan/Documents/robocup-ai/src')
from basic_skills.action import *
from basic_skills.move_to.move_to import *
from basic_skills.helper_functions import *
from matplotlib.widgets import Slider, Button, RadioButtons
from pygame_simulator.PySim import *

robot_rotation_speed = 4.25

class orbit_ball(action):
  #covers a pass from target_robot to target_loc
  def __init__(self, target_loc = False, offset = 125):
    action.__init__(self)
    self.pid = move_to()
    self.move_to = False
    self.target_loc = target_loc
    self.spiral_factor = .5
    self.speed_mod_factor = 4
    self.offset = offset
  def add(self, robot, game):
    self.robot = robot
    self.pid.robot = robot
    action.add(self, robot, game)
  def run(self):
    
    robot_vec = self.robot.loc - self.game.ball.loc
    robot_vec_scaled = robot_vec * self.offset / np.linalg.norm(robot_vec)
    target_loc = robot_vec_scaled * (1 - self.spiral_factor) + self.spiral_factor * robot_vec
    
    target_vec = self.game.ball.loc - self.target_loc
    current_angle = -math.atan2(robot_vec[1], robot_vec[0])
    target_angle = math.atan2(-target_vec[1], target_vec[0])
    rotation_angle = -min_angle(current_angle - target_angle) * (1 - self.spiral_factor) / 2
    #print(37, rotation_angle, current_angle, target_angle)
    orbit_vec = convert_local(target_loc, rotation_angle)
    move_to = orbit_vec + self.game.ball.loc
    speed_mod_vec = move_to - self.robot.loc
    move_to = move_to + speed_mod_vec * abs(rotation_angle) * self.speed_mod_factor
    
    rot_vec = convert_local(target_loc, -rotation_angle) * 2/3
    point_dir = (self.game.ball.loc + rot_vec) - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    self.pid.set_target(move_to, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.moving_to = move_to
    return actions
    
if __name__ == "__main__":
  game = PYsim(6)
  orbit_action = orbit_ball(np.array([0,3000]))
  move_action = move_to()
  game.blue_robots[0].add_action(orbit_action)
  game.yellow_robots[0].add_action(move_action)
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
    
    game.blue_robots[0].action.target_loc = game.yellow_robots[0].loc + game.yellow_robots[0].velocity * 5
        
    new_time = clock.tick()
    if game.yellow_robots[0].action.done():
      if i == 0:
        i = 1
      else:
        i = 0
      move_action.set_target(target_locs[i], 0)
    game.step()
    ttime = new_time