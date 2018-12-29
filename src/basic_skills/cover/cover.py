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

class cover(action):
  #covers a pass from target_robot to target_loc
  def __init__(self, target_loc, target_robot, interpose_factor = .5, min_interpose_offset = 0, interpose_weight = 25, lead_target_robot = True):
    action.__init__(self)
    self.pid = move_to()
    self.move_to = False
    self.target_loc = target_loc
    self.target_robot = target_robot
    
    self.lead_target_robot = lead_target_robot
    
    # factor from 0 to 1 describing how the robot will position itself between the
    # target_loc and target_robot
    # 1 is on target_loc
    # 0 is on target_robot
    self.interpose_factor = interpose_factor
    
    # minimum distance allowed between target_loc and robot
    # useful for body blocking
    self.min_interpose_offset = min_interpose_offset
    
    # controls how quickly the robot switches from quickest path to target loc movement
    self.interpose_weight = interpose_weight
  def add(self, robot, game):
    self.robot = robot
    self.pid.robot = robot
    action.add(self, robot, game)
  def run(self):
    shooting_pos = self.target_robot.loc
    if self.lead_target_robot:
      shooting_vec = self.target_loc - self.target_robot.loc
      delta_angle = abs(min_angle(self.target_robot.rot - math.atan2(shooting_vec[1], shooting_vec[0])))
      shooting_pos = shooting_pos + self.target_robot.velocity * delta_angle / robot_rotation_speed
    quickest_loc = drop_perpendicular(self.robot.loc, shooting_pos, shooting_vec)
    if np.linalg.norm(shooting_vec) * self.interpose_factor < self.min_interpose_offset:
      interpose_loc = shooting_pos + shooting_vec * self.min_interpose_offset / np.linalg.norm(shooting_vec)
    else:
      interpose_loc = shooting_pos + shooting_vec * self.interpose_factor
    quickest_distance = np.linalg.norm(quickest_loc - self.robot.loc)
    move_to = ((quickest_loc * quickest_distance + interpose_loc * self.interpose_weight)
                  / (self.interpose_weight + quickest_distance))
    
    point_dir = self.target_robot.loc - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    self.pid.set_target(move_to, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.move_to = move_to
    return actions
    
class cover_ball(action):
  # Covers a pass from target_ball to target_loc
  # for goalie
  # identical to cover with "lead_target_robot" False
  def __init__(self, target_loc, target_ball, interpose_factor = .5, min_interpose_offset = 0, interpose_weight = 25):
    action.__init__(self)
    self.pid = move_to()
    self.move_to = False
    self.target_loc = target_loc
    self.target_ball = target_ball
    
    # factor from 0 to 1 describing how the robot will position itself between the
    # target_loc and target_ball
    # 1 is on target_loc
    # 0 is on target_ball
    self.interpose_factor = interpose_factor
    
    # minimum distance allowed between target_loc and robot
    # useful for body blocking
    self.min_interpose_offset = min_interpose_offset
    
    # controls how quickly the robot switches from quickest path to target loc movement
    self.interpose_weight = interpose_weight
  def add(self, robot, game):
    self.robot = robot
    self.pid.robot = robot
    action.add(self, robot, game)
  def run(self):
    shooting_pos = self.target_ball.loc
    shooting_vec = self.target_loc - self.target_ball.loc
    quickest_loc = drop_perpendicular(self.robot.loc, shooting_pos, shooting_vec)
    if np.linalg.norm(shooting_vec) * self.interpose_factor < self.min_interpose_offset:
      interpose_loc = shooting_pos + shooting_vec * self.min_interpose_offset / np.linalg.norm(shooting_vec)
    else:
      interpose_loc = shooting_pos + shooting_vec * self.interpose_factor
    quickest_distance = np.linalg.norm(quickest_loc - self.robot.loc)
    move_to = ((quickest_loc * quickest_distance + interpose_loc * self.interpose_weight)
                  / (self.interpose_weight + quickest_distance))
    
    point_dir = self.target_ball.loc - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    self.pid.set_target(move_to, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.move_to = move_to
    return actions
    
class cover_robots(action):
  # Covers a pass from target_bot1 to target_bot2
  def __init__(self, target_bot1, target_bot2, interpose_factor = .5, min_interpose_offset = 190, interpose_weight = 25, lead_target_robot = True):
    action.__init__(self)
    self.pid = move_to()
    self.move_to = False
    self.target_bot1 = target_bot1
    self.target_bot2 = target_bot2
    self.lead_target_robot = lead_target_robot
    
    # factor from 0 to 1 describing how the robot will position itself between the
    # target_bot1 and target_bot2
    # 1 is on target_bot1
    # 0 is on target_bot2
    self.interpose_factor = interpose_factor
    
    # minimum distance allowed between target_bot1 and robot
    # useful for body blocking
    self.min_interpose_offset = min_interpose_offset
    
    # controls how quickly the robot switches from quickest path to target loc movement
    self.interpose_weight = interpose_weight
  def add(self, robot, game):
    self.robot = robot
    self.pid.robot = robot
    action.add(self, robot, game)
  def run(self):
    shooting_pos = self.target_bot2.loc
    target_loc = self.target_bot1.loc
    shooting_vec = self.target_bot1.loc - self.target_bot2.loc
    if self.lead_target_robot:
      if type(self.target_bot1) is robot:
        delta_angle = abs(min_angle(self.target_bot1.rot - math.atan2(shooting_vec[1], shooting_vec[0])))
        target_loc = target_loc + self.target_bot1.velocity * delta_angle / robot_rotation_speed
      
      if type(self.target_bot2) is robot:
        delta_angle = abs(min_angle(self.target_bot2.rot - math.atan2(shooting_vec[1], shooting_vec[0])))
        shooting_pos = shooting_pos + self.target_bot2.velocity * delta_angle / robot_rotation_speed
    quickest_loc = drop_perpendicular(self.robot.loc, shooting_pos, shooting_vec)
    if np.linalg.norm(shooting_vec) * self.interpose_factor < self.min_interpose_offset:
      interpose_loc = shooting_pos + shooting_vec * self.min_interpose_offset / np.linalg.norm(shooting_vec)
    else:
      interpose_loc = shooting_pos + shooting_vec * self.interpose_factor
    quickest_distance = np.linalg.norm(quickest_loc - self.robot.loc)
    move_to = ((quickest_loc * quickest_distance + interpose_loc * self.interpose_weight)
                  / (self.interpose_weight + quickest_distance))
    
    point_dir = self.target_bot2.loc - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    self.pid.set_target(move_to, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.move_to = move_to
    return actions

if __name__ == "__main__":
  game = PYsim(6)
  cover_action = cover(np.array([0,3000]), game.yellow_robots[0], 1, min_interpose_offset = 50)
  move_action = move_to()
  game.add_action(cover_action, 0, True)
  game.add_action(move_action, 0, False)
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
    new_time = clock.tick()
    if game.yellow_robots[0].action.done():
      if i == 0:
        i = 1
      else:
        i = 0
      move_action.set_target(target_locs[i], 0)
    game.step()
    ttime = new_time