import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/home/dpyryev/Desktop/robocup_ai/robocup-ai/src')
from basic_skills.action import *
from basic_skills.move_to.move_to import *
from basic_skills.helper_functions import *

robot_rotation_speed = 4.25

class ball_orbit(action):
  #covers a pass from target_robot to target_loc
  def __init__(self, target_loc = False, offset = 125):
    action.__init__(self)
    self.pid = move_to()
    self.move_to = False
    self.target_loc = target_loc
    self.spiral_factor = .35
    self.speed_mod_factor = 4
    self.offset = offset
  def add(self, robot, game):
    self.robot = robot
    self.pid.robot = robot
    self.game = game
    action.add(self, robot, game)
  def run(self):
    ball_extrapolation = self.game.ball.loc + self.game.ball.velocity/10
    robot_vec = self.robot.loc - ball_extrapolation
    #print(self.robot.loc, ball_extrapolation)
    robot_vec_scaled = robot_vec * self.offset / np.linalg.norm(robot_vec)
    target_loc = robot_vec_scaled * (1 - self.spiral_factor) + self.spiral_factor * robot_vec
    
    target_vec = ball_extrapolation - self.target_loc
    current_angle = -math.atan2(robot_vec[1], robot_vec[0])
    target_angle = math.atan2(-target_vec[1], target_vec[0])
    rotation_angle = -min_angle(current_angle - target_angle) * (1 - self.spiral_factor) / 2
    #print(37, rotation_angle, current_angle, target_angle)
    orbit_vec = convert_local(target_loc, rotation_angle)
    move_to = orbit_vec + ball_extrapolation
    speed_mod_vec = move_to - self.robot.loc
    move_to = move_to + speed_mod_vec * abs(rotation_angle) * self.speed_mod_factor
    
    rot_vec = convert_local(target_loc, -rotation_angle) * .85
    point_dir = (ball_extrapolation + rot_vec) - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    self.pid.set_target(move_to, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.moving_to = move_to
    return actions
    
if __name__ == "__main__":
  game = GRsim(6)
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
