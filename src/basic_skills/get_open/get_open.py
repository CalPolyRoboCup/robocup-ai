import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/Users/nathan/Documents/robocup-ai/src')
from basic_skills.action import *
from basic_skills.helper_functions import *
from basic_skills.action import *
from basic_skills.move_to.move_to import *

from pygame_simulator.PySim_noise import *
from basic_skills.ball_interception.Ball_Interception import *
from basic_skills.ball_interception.Catch_Pygym import *

def worst_intercept(location, p, enemies):
  '''
  brief: finds the enemy best able to intercept a pass from location to pangle
  params:
    location - origin of pass
    p - target of pass
    enemies - list of enemies
  returns:
    worst - calculated margin of pass (lower is worse, more easily intercepted). 
            Not a straight distance includes travel time (and TODO current velocity)
    worst_local - the position of the enemy best able to intercept the pass in local 
                  coordinates. [distance along pass, distance perpendicular to pass]
                  so large first component means the enemy is farther from location
                  and closer to p, and large (+ or -) second component means the enemy is
                  farther from the line of the pass     
    pangle - angle of the pass from the +x axis.
             convert_local(worst_local, pangle) + location 
             gets you back to normal coordinate system
  '''
  ball_to_robot_speed = 10
  worst = -1
  first = True
  worst_local = np.array([0,0])
  pvec = np.array(p) - np.array(location)
  pangle = -math.atan2(pvec[1], pvec[0])
  pmag = np.linalg.norm(pvec)
  for e in enemies:
    local = convert_local(e.loc - location, -pangle)
    if local[0] < 0:
      dist = np.linalg.norm(local)
      dist += dist/ball_to_robot_speed
    elif local[0] > pmag:
      dist = np.linalg.norm(local - np.array([pmag, 0]))
      dist -= dist/ball_to_robot_speed
    else:
      dist = abs(local[1])
      dist -= local[0]/ball_to_robot_speed
    if first or dist < worst:
      worst = dist
      worst_local = local
      first = False
      worst_loc = e.loc
    
  if worst < 0:
    worst = 0
  return worst, worst_local, pangle

class get_open(action):
  '''
  brief: attempts to move the robot to a location that is capable of passing to each point in points
        points are weighted by weights
        
        If the current location is bad or another robot with the same "task" is to close the robot will
        move to a new location
  '''
  def __init__(self, points, weights, enemies, allies):
    action.__init__(self)
    self.points = points
    self.weights = weights
    self.enemies = enemies
    self.allies = allies
    self.move_to = np.array([0,0])
    
    self.stand_off_distance = 750
    self.freakout_weight = 150
    self.max_catch_dist = 700
    self.samples = 240
    
    #smaller values stay farther from enemy robots
    self.depth_weight = .015
    
    #for debugging
    self.prints = [(np.array([0,0]), 1), (np.array([0,0]), 1)]#, np.array([0,0]), np.array([0,0]), np.array([0,0]), np.array([0,0]), np.array([0,0])]
    
    self.dart = False
    self.dart_to = None
    
    self.current_score = 0

    self.pid = move_to()
  def add(self, robot, game):
    self.robot = robot
    self.pid.robot = robot
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
  def get_better_loc(self):
    '''
    brief: samples several points and returns the one with the highest "score" by a metric from rate_point
    params: None
    returns: better location, better score
    '''
    first = True
    target_point = self.robot.loc*np.array([1,-1])
    for i in range(self.samples):
      sample_point = np.random.uniform(-1, 1, size = [2])*np.array([2000, 2000]) + target_point
      _, score = self.rate_point(sample_point)
      for a in self.allies:
        if np.linalg.norm(a.loc - sample_point) < self.stand_off_distance*2:
          score -= 500
      if first or score > best:
        best = score
        best_point = sample_point
        first = False
    return best_point, best
    # vect = self.robot.loc - a.loc
    # vect = 1000 * vect / np.linalg.norm(vect)
    # return a.loc + vect
  def run(self):
    #if we are too close to an allie dart
    if not self.dart:
      for a in self.allies:
        if (a.id != self.robot.id and a.task == self.robot.task and type(a.action) == get_open and 
            a.action.dart == 0 and np.linalg.norm(a.loc - self.robot.loc) < self.stand_off_distance and 
            self.current_score < a.action.current_score):
          self.dart = 120
          self.dart_to, score = self.get_better_loc()
          #print("freak something touched me", self.robot.id, self.robot.loc, self.dart_to, score, self.current_score, a.action.current_score)
          break
          
    #while darting move to "better loc"
    if self.dart > 0:
      #print(self.robot.id, "darting")
      move_to = self.dart_to
      if self.pid.done() and self.dart != 120:
        print("finished", self.robot.loc, self.robot.id)
        self.dart = 1
      self.dart -= 1
      
    #if we aren't darting do the normal thing
    if self.dart == 0:
      improvements, score = self.rate_point(self.robot.loc)
      
      #if we aren't doing the normal thing well dart
      if score < self.freakout_weight:
        self.dart = 120
        self.dart_to, score = self.get_better_loc()
        #print("freak I'm scared", self.robot.id, self.robot.loc, self.dart_to, self.current_score, score)
      self.current_score = score
      move_to = improvements * .3 + self.move_to * .7
    
    
    point_dir = self.points[0] - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
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
  for i in game.yellow_robots[1:3]:
    guard_locs = [game.yellow_robots[0].loc]
    # for y in game.yellow_robots:
      # if y.id != i.id:
        # guard_locs.append(y.loc)
    guard_locs.append([-5100,0])
    game.add_action(get_open(guard_locs, [1.3,0.4], game.blue_robots, game.yellow_robots), i.id, False)
    ind += 1
    if ind == max_bots_per_team:
      ind = 1
  ind = 1
  for i in game.yellow_robots[3:5]:
    i.task = ind
    game.add_action(get_open([game.yellow_robots[0].loc, game.yellow_robots[ind].loc], [1,1], game.blue_robots, game.yellow_robots), i.id, False)
    ind += 1
  while 1:
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
    game.step(key_points = game.yellow_robots[1].action.prints)
    ttime = new_time