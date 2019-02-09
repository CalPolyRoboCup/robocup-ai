import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '../..')
from basic_skills.action import *
from basic_skills.helper_functions import *
from basic_skills.action import *
from basic_skills.move_to.move_to import *
from basic_skills.dribble_ball.dribble_ball import *
from basic_skills.get_open.get_open import *

from pygame_simulator.PySim_noise import *

class get_open_with_ball(action):
  def __init__(self, points, weights, enemies):
    action.__init__(self)
    self.points = points
    self.weights = weights
    self.enemies = enemies
    self.move_to = np.array([0,0])
    
    self.stand_off_distance = 750
    self.stand_off_weight = 750;
    self.max_catch_dist = 700
    self.depth_weight = .3
    self.prints = [(np.array([0,0]), 1), (np.array([0,0]), 1), np.array([0,0]), np.array([0,0]), np.array([0,0]), np.array([0,0]), np.array([0,0])]

    self.pid = dribble_ball()
  def add(self, robot, game):
    #print("3555")
    self.pid.add(robot, game)
    #print(self.pid.pid.robot, type(self.pid))
    action.add(self, robot, game)
  def rate_point(self, location):
    '''
    brief: checks passes from location to self.points and generates a rating
    params: location to rate
    returns: 
      improvements - a point that should be better than location
      score - a score for location
    '''
    improvements = np.array([0,0])
    improv_weight = 0
    pind = 0
    score = 0
    for p in self.points:
      worst, worst_local, pangle = worst_intercept(location, p, self.enemies)
      importance = abs(20000/(100+worst))
      if worst_local[1] < 0:
        improvement = np.array([worst_local[0]*self.depth_weight, importance])
      else:
        improvement = np.array([worst_local[0]*self.depth_weight, -importance])
      if np.linalg.norm(improvement) < self.stand_off_distance:
        improvement = improvement * self.stand_off_distance / np.linalg.norm(improvement)
      improvement_field = convert_local(improvement, pangle) + location
      improvements = improvements + improvement_field * importance * self.weights[pind]
      improv_weight += importance * self.weights[pind]
      score += worst * self.weights[pind]
      
      #debugging
      # if pind == 0:
        # self.prints[pind] = (improvement_field, self.weights[pind]*10)
      # else:
        # self.prints[pind] = (improvement_field, self.weights[pind]*-10)
      if pind == 0:
        self.prints[pind] = (convert_local(worst_local, pangle) + location, self.weights[pind]*10)
      else:
        self.prints[pind] = (convert_local(worst_local, pangle) + location, self.weights[pind]*-10)
        
      pind += 1
    
    #weighted average of improvements to passes to each point
    improvements = improvements/improv_weight
    
    #don't stand too close to one of the points
    for p in self.points:
      push_out = improvements - p
      push_out_mag = np.linalg.norm(push_out)
      if push_out_mag < self.stand_off_distance:
        improvements = improvements + push_out / push_out_mag * self.stand_off_distance
    return improvements, score
  def run(self):
    improvements, score = self.rate_point(self.robot.loc)
    self.current_score = score
    #print(self.robot.id, self.robot.loc, improv_weight)
    move_to = improvements
    for e in self.enemies:
      vec = e.loc - self.robot.loc
      vec_mag = np.linalg.norm(vec)
      if vec_mag < self.stand_off_distance:
        vec = vec * self.stand_off_weight / np.linalg.norm(vec)
        move_to -= vec
    
    
    if np.linalg.norm(move_to - self.robot.loc) < 300:
      self.pid.offset = 120
      point_dir = self.points[0] - self.robot.loc
      target_rot = -math.atan2(point_dir[1], point_dir[0])
      self.pid.set_target(self.points[0])
      actions = self.pid.run()
      self.actions = actions
      self.move_to = move_to
    else:
      self.pid.offset = 90
      self.pid.set_target(move_to)
      actions = self.pid.run()
      self.actions = actions
      self.move_to = move_to
    return actions
    
    

#complicated test code
#this isn't final
if __name__ == "__main__":
  max_bots_per_team = 6
  game = PYsim(max_bots_per_team)
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  ind = 2
  allie_locs = []
  allie_weights = []
  for y in [game.yellow_robots[1]]:
    allie_locs.append(y.loc)
    allie_weights.append(1)
  game.add_action(get_open_with_ball(allie_locs, allie_weights, game.blue_robots), 0, False)
  for i in game.yellow_robots[1:3]:
    guard_locs = [game.yellow_robots[0].loc]
    # for y in game.yellow_robots:
      # if y.id != i.id:
        # guard_locs.append(y.loc)
    guard_locs.append([-5100,0])
    game.add_action(get_open(guard_locs, [1.3,0.2], game.blue_robots, game.yellow_robots), i.id, False)
    ind += 1
    if ind == max_bots_per_team:
      ind = 1
  ind = 1
  for i in game.yellow_robots[3:5]:
    i.task = ind
    game.add_action(get_open([game.yellow_robots[0].loc, game.yellow_robots[ind].loc], [1,.5], game.blue_robots, game.yellow_robots), i.id, False)
    ind += 1
  while 1:
    allie_locs = []
    for y in [game.yellow_robots[1]]:
      allie_locs.append(y.loc)
    game.yellow_robots[0].action.points = allie_locs
    
    ind = 1
    while ind != max_bots_per_team - 1:
      game.yellow_robots[ind].action.points[0] = game.yellow_robots[0].loc
      if ind >= 3 and ind < 5:
        game.yellow_robots[ind].action.points[1] = game.yellow_robots[ind - 2].loc
      ind += 1
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
      if event.type == MOUSEBUTTONDOWN:
        pressed1, pressed2, pressed3 = pygame.mouse.get_pressed()
        #left mouse button
        if pressed1:
          print("high")
          game.yellow_robots_internal[1].loc = game.convert_to_field_position(pygame.mouse.get_pos())
        #right mouse button
        if pressed3:
          print("hiiii")
          game.yellow_robots_internal[0].loc = game.convert_to_field_position(pygame.mouse.get_pos())
      if event.type == KEYDOWN or event.type == KEYUP:
        keys = pygame.key.get_pressed()
        #press r-key to reset
        if keys[K_r]:
          game.reset()
    new_time = clock.tick()
    #print("kp", game.yellow_robots[0].action.move_to)
    game.step(key_points = (game.yellow_robots[0].action.move_to, 3))
    ttime = new_time