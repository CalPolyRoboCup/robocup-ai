import numpy as np
import math
import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/../../..')
from basic_skills.src.basic_skills.helper_functions import *
from basic_skills.src.basic_skills.action import *
from basic_skills.src.basic_skills.move_to import *
from basic_skills.src.basic_skills.dribble_ball import *

from pysim.src.pysim.PySim import *

def worst_intercept(location, p, enemies):
  '''
  brief: finds the enemy best able to intercept a pass from location to p
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
  ball_to_robot_speed = 15
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

class get_open(MoveTo):
  '''
  brief: attempts to move the robot to a location that is capable of passing to each point in points
        points are weighted by weights
        
        If the current location is bad or another robot with the same "task" is to close the robot will
        move to a new location
  '''
  def __init__(self, points, weights, enemies, allies, game):
    '''
    params: points - points to optimize passes to
            weights - relative importance of points. Must be the same size as points
            enemies - list of enemy robots. Used to calculated pass values
            allies - list of allied robots. Used to ensure robots spread out.
    '''
    MoveTo.__init__(self, game)
    self.points = points
    self.weights = weights
    self.enemies = enemies
    self.allies = allies
    self.move_to = np.array([0,0])
    self.game = game
    
    '''
    magic numbers
    stand_off_distance - distance to maintain between allies
    freakout_weight - minimum acceptable score. If score drops bellow this the robot repositions
    samples - number of points to consider when repositioning
    sample_points - the points to consider when repositioning
    speed_mod - increases the speed of the robot. Can also cause oscillations
    depth_weight - smaller values stay farther from enemy robots. Not sure how to describe this without 
                    references to the code. See rate_point function and calculations of the improvement to the point.
    '''
    self.stand_off_distance = 750
    self.freakout_weight = 140
    self.samples = 12
    self.sample_points = [[x,y] for x in [-4000, -2000, 2000, 4000] for y in [-3000, -1500, 1500, 3000]]
    self.speed_mod = 2
    self.depth_weight = .015
    
    # for debugging. Keypoints to plot with pygame.step
    self.prints = [(np.array([0,0]), 1), (np.array([0,0]), 1)]#, np.array([0,0]), np.array([0,0]), np.array([0,0]), np.array([0,0]), np.array([0,0])]
    
    # when we reposition we also call it darting
    self.dart = False
    self.dart_to = None
    
    # score representing how well this robot can make passes to points
    self.current_score = 0
    self.lag_loc = np.array([0,0])
    
  def add(self, robot):
    Action.add(self, robot)
    self.lag_loc = robot.loc
    
  def rate_point(self, location):
    '''
    brief: checks passes from location to self.points and generates a rating
    params: location to rate
    returns: 
      improvements - a point that should be better than location
      score - a score for location. Higher is better
    '''
    improvements = np.array([0,0])
    improv_weight = 0
    pind = 0
    score = 0
    for p in self.points:
    
      '''
      push location away from the worst intercepting enemy
      we scale down the local X dimension by depth_weight because we don't care as much about it 
      as the local Y dimension
      We push points farther if they are badly intercepted. (a free pass is pushed barely at all)
      
      We will then make a weighted average of these points and return it as an improvement of location
      '''
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
      
      # give score for good passes
      # cap available score from a single pass so robots don't run back as much
      if worst > 500:
        worst = 500
      score += worst * self.weights[pind]
      
      if pind == 0:
        self.prints[pind] = (convert_local(worst_local, pangle) + location, self.weights[pind]*10)
      else:
        self.prints[pind] = (convert_local(worst_local, pangle) + location, self.weights[pind]*-10)
        
      pind += 1
    
    # weighted average of improvements to passes to each point
    improvements = improvements/improv_weight
    
    # don't stand too close to one of the points
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
    target_point = self.points[0]
    for i in range(self.samples):
      sample_point = self.sample_points[i]#
      for i in range(25):
        improvement, score = self.rate_point(sample_point)
        sample_point = (improvement - sample_point) + sample_point
      _, score = self.rate_point(sample_point)
      
      
      for a in self.allies:
        score -= 50000/(100 + np.linalg.norm(a.loc - sample_point))
      if first or score > best:
        best = score
        best_point = sample_point
        first = False
    return best_point, best
    
  def run(self, delta_time):
    # while darting move to "better loc"
    if self.dart > 0:
      move_to = self.dart_to
      self.current_score = 0
      if self.pid.done() and self.dart != 120:
        self.dart = 1
      self.dart -= 1
      
    # if we aren't darting do the normal thing
    if self.dart == 0:
    
      # we calculate based on a lagging version of our location to prevent oscillations between two enemies
      improvement, score = self.rate_point(self.lag_loc)
      # if we aren't doing well
      # dart
      if score < self.freakout_weight:
        self.dart = 120
        self.dart_to, score = self.get_better_loc()
      self.current_score = score
      move_to = self._robot.loc + self.speed_mod*(improvement - self._robot.loc)
    
    self.lag_loc = self._robot.loc * .1 + .9 * self.lag_loc
    
    point_dir = self.points[0] - self._robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    self.set_target(move_to, target_rot)
    Actions = MoveTo.run(self, delta_time)
    self.Actions = Actions
    self.move_to = move_to
    return Actions
    
'''
forward support class. Gets open for ball controler and shot at the goal
'''
class striker(get_open):
  def __init__(self, ball_controller, goal, enemies, allies, game):
    self.goal = goal
    self.ball_controller = ball_controller
    self.allies = allies
    self.enemies = enemies
    get_open.__init__(self, [ball_controller.loc, goal], [1, 0.4], enemies, allies, game)
    
  def add(self, robot):
    robot.task = 1
    get_open.add(self, robot)
    
  def run(self, delta_time):
    self.points[0] = self.ball_controller.loc
    return get_open.run(self, delta_time)
    
'''
Rear support class. Gets open for ball_controller and anouther allied robot (currently one of the strikers
'''
class fielder(get_open):
  def __init__(self, ball_controller, target_to_support, enemies, allies, game):
    self.target_to_support = target_to_support
    self.ball_controller = ball_controller
    self.allies = allies
    self.enemies = enemies
    get_open.__init__(self, [ball_controller.loc, target_to_support.loc], [1, 0.7], enemies, allies, game)
    
  def add(self, robot):
    robot.task = 2
    get_open.add(self, robot)
    
  def run(self, delta_time):
    self.points[1] = self.target_to_support.loc
    self.points[0] = self.ball_controller.loc
    return get_open.run(self, delta_time)
    
'''
rewraps get_open, but with dribble_ball instead of move_to for control and no darting around.
Best used with only a single support_robot.
'''
class get_open_with_ball(get_open):
  def __init__(self, support_robots, enemies, allies):
    '''
    breif: create an object that tries to move with the ball to get open for 
    '''
    get_open.__init__(self, [a.loc for a in support_robots], [1 for a in support_robots], enemies, allies)
    self.allies = allies
    self.enemies = enemies
    self.support_robots = support_robots
    self.pid = dribble_ball()
    
    # change default movement calculation. 
    # Original description:
    # depth_weight - smaller values stay farther from enemy robots. Not sure how to describe this without 
    #                references to the code. See rate_point function and calculations of the improvement to the point.
    self.depth_weight = 0.025
    
    # do not dart because of bad positions. Darting causes us to drop the ball.
    self.freakout_weight = -10
    
  def add(self, robot):
    # setting the robot task is important so that dart doesn't trigger from being too close to a robot with the same task
    # ideally we would remove all dart checking, but I think this wrapper is neater 
    robot.task = 3
    get_open.add(self, robot)
    
  def run(self, delta_time):
    for i in range(len(self.support_robots)):
      self.points[i] = self.support_robots[i].loc
    return get_open.run(self, delta_time)