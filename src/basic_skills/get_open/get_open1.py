import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/Users/nathan/Documents/robocup-ai/src')
from basic_skills.action import *
from basic_skills.helper_functions import *
from basic_skills.action import *
from basic_skills.move_to.move_to import *

from pygame_simulator.PySim import *
from basic_skills.ball_interception.Ball_Interception import *
from basic_skills.ball_interception.Catch_Pygym import *

class get_open(action):
  def __init__(self, points, weights, enemies, allies):
    action.__init__(self)
    self.points = points
    self.weights = weights
    self.enemies = enemies
    self.allies = allies
    self.move_to = np.array([0,0])
    
    self.stand_off_distance = 750
    self.freakout_weight = 1750
    self.max_catch_dist = 700
    self.samples = 180
    #don't touch this
    self.dart = False
    self.dart_to = None

    self.pid = move_to()
  def add(self, robot, game):
    self.robot = robot
    self.pid.robot = robot
    action.add(self, robot, game)
  def rate_point(self, location):
    improvements = np.array([0,0])
    improv_weight = 0
    pind = 0
    score = 0
    for p in self.points:
      ball_to_robot_speed = 1.5
      worst = -1
      worst_local = np.array([0,0])
      pvec = np.array(p) - np.array(location)
      pangle = -math.atan2(pvec[1], pvec[0])
      pmag = np.linalg.norm(pvec)
      for e in self.enemies:
        local = convert_local(e.loc - location, pangle)
        if local[0] < 0:
          dist = np.linalg.norm(local)
          dist += dist/ball_to_robot_speed
        elif local[0] > pmag:
          dist = np.linalg.norm(local - np.array([pmag, 0]))
          dist -= local[0]/ball_to_robot_speed
        else:
          dist = local[1]
          dist -= local[0]/ball_to_robot_speed
        if worst < 0 or dist < worst:
          worst = dist
          worst_local = local
        
      if worst < 0:
        worst = 0
      if worst > self.max_catch_dist:
        worst = self.max_catch_dist
        
      importance = abs(5000/(100+worst))
      if worst_local[1] < 0:
        improvement = np.array([worst_local[0], importance])
      else:
        improvement = np.array([worst_local[0], -importance])
      if np.linalg.norm(improvement) < self.stand_off_distance:
        improvement = improvement * self.stand_off_distance / np.linalg.norm(improvement)
      improvement_field = convert_local(improvement, -pangle) + location
      #print(improvement_field, improvement, convert_local(improvement, -pangle), worst_loc, importance * self.weights[pind], pangle)
      improvements = improvements + improvement_field * importance * self.weights[pind]
      improv_weight += importance * self.weights[pind]
      pind += 1
      score += worst * self.weights[pind]
    improvements = improvements/improv_weight
    return improvements, score
  def get_better_loc(self, a):
    # first = True
    # for i in range(self.samples):
      # sample_point = np.random.uniform(-1, 1, size = [2])*np.array([2000, 2000]) + self.robot.loc
      # _, score = self.rate_point(sample_point)
      # for a in self.allies:
        # if np.linalg.norm(a.loc - sample_point) < self.stand_off_distance*2:
          # score -= 500
      # if first or score > best:
        # best = score
        # best_point = sample_point
        # first = False
    # return best_point, best
    vect = self.robot.loc - a.loc
    vect = 1000 * vect / np.linalg.norm(vect)
    return a.loc + vect
  def run(self):
    if not self.dart:
      for a in self.allies:
        if a.id != self.robot.id and type(a.action) == get_open and a.action.dart == 0 and np.linalg.norm(a.loc - self.robot.loc) < self.stand_off_distance:
          self.dart = 40
          self.dart_to = self.get_better_loc(a)
          print("freak", self.robot.id, self.robot.loc, self.dart_to)
          break
    if self.dart > 0:
      #print(self.robot.id, "darting")
      self.dart -= 1
      move_to = self.dart_to
      if self.pid.done():
        #print("finished", self.robot.loc, self.robot.id)
        self.dart = 0
    if self.dart == 0:
      improvements, improv_weight = self.rate_point(self.robot.loc)
      # if self.robot.loc[0] < -4000:
        # print(improvements)
      # if improv_weight > self.freakout_weight:
        # self.dart = 240
        # self.dart_to, score = self.get_better_loc()
        # print("freak", self.robot.id, self.robot.loc, self.dart_to, score, improv_weight)
      #print(self.robot.id, self.robot.loc, improv_weight)
      move_to = improvements
    
    
    point_dir = self.points[0] - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    print(move_to)
    self.pid.set_target(move_to, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.move_to = move_to
    return actions
    
    

if __name__ == "__main__":
  max_bots_per_team = 6
  game = PYsim(max_bots_per_team)
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  ind = 2
  for i in game.yellow_robots[1:]:
    other_bots = []
    for y in game.yellow_robots:
      if y.id != i.id:
        other_bots.append(y.loc)
    other_bots.append([-5100,0])
    i.add_action(get_open(other_bots, [1,.05,.1,.05,.1,.05,.3], game.blue_robots, game.yellow_robots))
    ind += 1
    if ind == max_bots_per_team:
      ind = 1
  # ind = 1
  # for i in game.yellow_robots[3:5]:
    # other_bots = []
    # for y in game.yellow_robots:
      # if y.id != i.id:
        # other_bots.append(y.loc)
    # other_bots.append([-5100,0])
    # i.add_action(get_open([game.yellow_robots[0].loc, game.yellow_robots[ind].loc], [1,1], game.blue_robots, game.yellow_robots))
    # ind += 1
  while 1:
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
    new_time = clock.tick()
    game.step()
    ttime = new_time