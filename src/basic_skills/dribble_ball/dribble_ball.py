import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/Users/nathan/Documents/robocup-ai/src')
from basic_skills.action import *
from basic_skills.move_to.move_to import *
from basic_skills.helper_functions import *
from matplotlib.widgets import Slider, Button, RadioButtons
from pygame_simulator.PySim_noise import *

robot_rotation_speed = 4.25
'''
this is basically a reskin of orbit ball with parameters tuned so that it pushes the ball instead of orbiting it
'''
class dribble_ball(action):
  #covers a pass from target_robot to target_loc
  def __init__(self, target_loc = False, offset = 90):
    action.__init__(self)
    self.pid = move_to()
    self.moving_to = False
    self.target_loc = target_loc
    self.spiral_factor = .4
    self.push_speed_factor = 1
    self.chase_down_factor = 4
    self.speed_mod_factor = 7
    self.offset = offset
  def set_target(self, target_loc):
    self.target_loc = target_loc
  def add(self, robot, game):
    #print("2999")
    self.pid.add(robot, game)
    action.add(self, robot, game)
  def run(self):
    ball_extrapolation = self.game.ball.loc# + self.game.ball.velocity/10
    robot_vec = self.robot.loc - ball_extrapolation
    #print(self.robot.loc, ball_extrapolation)
    robot_vec_scaled = robot_vec * self.offset / np.linalg.norm(robot _vec)
    target_loc = robot_vec_scaled * (1 - self.spiral_factor) + self.spiral_factor * robot_vec
    
    target_vec = ball_extrapolation - self.target_loc
    current_angle = -math.atan2(robot_vec[1], robot_vec[0])
    target_angle = math.atan2(-target_vec[1], target_vec[0])
    rotation_angle = -min_angle(current_angle - target_angle) * (1 - self.spiral_factor) / 2
    #print(37, rotation_angle, current_angle, target_angle)
    orbit_vec = convert_local(target_loc, rotation_angle)
    move_to = orbit_vec + ball_extrapolation
    speed_mod_vec = move_to - self.robot.loc
    #print(abs(rotation_angle))
    on_angle_factor = np.clip(math.pi - 10*abs(rotation_angle), 0, math.pi)**2
    off_angle_factor = np.clip(-math.pi/20 + abs(rotation_angle), 0, math.pi)**2
    #print(on_angle_factor, off_angle_factor, np.linalg.norm(speed_mod_vec))
    if np.linalg.norm(speed_mod_vec) < 300:
      move_to = move_to + speed_mod_vec * (self.speed_mod_factor + self.push_speed_factor * on_angle_factor
                + off_angle_factor * self.chase_down_factor)
    
    point_dir = (ball_extrapolation) - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    self.pid.set_target(move_to, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.moving_to = move_to
    return actions
    
if __name__ == "__main__":
  game = PYsim(6)
  dribble_action = dribble_ball(np.array([0,3000]))
  game.add_action(dribble_action, 0, True)
  #game.add_action(move_action, 0, False)
  i = 0
  
  target_loc = np.array([0,3000])
  time = 0
  
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  while 1:
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
    
    time += 1
    if time % 300 == 0:
      target_loc = np.random.uniform(-1, 1, size = [2]) * np.array([4000, 2000])
    game.blue_robots[0].action.target_loc = target_loc
      
        
    new_time = clock.tick()
    if time != 1:
      kp = [target_loc, (game.blue_robots[0].action.moving_to, -3)]
    else:
      kp = [target_loc]
    game.step(key_points = kp)
    ttime = new_time