import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '../..')
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
    '''
    params: points - points to optimize passes to
            weights - relative importance of points. Must be the same size as points
            enemies - list of enemy robots. Used to calculated pass values
            allies - list of allied robots. Used to ensure robots spread out.
    '''
    action.__init__(self)
    self.points = points
    self.weights = weights
    self.enemies = enemies
    self.allies = allies
    self.move_to = np.array([0,0])
    
    '''
    magic numbers
    stand_off_distance - distance to maintain between allies
    freakout_weight - minimum acceptable score. If score drops bellow this the robot repositions
    samples - number of points to consider when repositioning
    sample_points - the points to consider when repositioning
    speed_mod - increases the speed of the robot. Can also cause oscillations
    depth_weight - smaller values stay farther from enemy robots. Not sure how to describe this without references to the code.
    '''
    self.stand_off_distance = 750
    self.freakout_weight = 300
    self.samples = 12
    self.sample_points = [[x,y] for x in [-4000, -2000, 2000, 4000] for y in [-3000, -1500, 1500, 3000]]
    self.speed_mod = 2
    self.depth_weight = .015
    
    #for debugging. Keypoints to plot with pygame.step
    self.prints = [(np.array([0,0]), 1), (np.array([0,0]), 1)]#, np.array([0,0]), np.array([0,0]), np.array([0,0]), np.array([0,0]), np.array([0,0])]
    
    #when we reposition we also call it darting
    self.dart = False
    self.dart_to = None
    
    #score representing how well this robot can make passes to points
    self.current_score = 0

    self.pid = move_to()
  def add(self, robot, game):
    action.add(self, robot, game)
    action.add(self.pid, robot, game)
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
      
      #give score for good passes
      #cap available score from a single pass so robots don't run back as much
      if worst > 500:
        worst = 500
      score += worst * self.weights[pind]
      
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
    target_point = self.points[0]
    for i in range(self.samples):
      sample_point = self.sample_points[i]#
      for i in range(10):
        improvement, score = self.rate_point(sample_point)
        sample_point = (improvement - sample_point)*55 + sample_point
      _, score = self.rate_point(sample_point)
      
      
      for a in self.allies:
        if np.linalg.norm(a.loc - sample_point) < self.stand_off_distance*3 and self.robot.task == a.task:
          score -= 500
      if first or score > best:
        best = score
        best_point = sample_point
        first = False
    return best_point, best
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
      move_to = self.dart_to
      if self.pid.done() and self.dart != 120:
        print("finished", self.robot.loc, self.robot.id)
        self.dart = 1
      self.dart -= 1
      
    #if we aren't darting do the normal thing
    if self.dart == 0:
      improvement, score = self.rate_point(self.robot.loc)
      #if we aren't doing well
      #dart
      if score < self.freakout_weight:
        self.dart = 120
        self.dart_to, score = self.get_better_loc()
        #print("freak I'm scared", self.robot.id, self.robot.loc, self.dart_to, self.current_score, score)
      self.current_score = score
      move_to = self.robot.loc + self.speed_mod*(improvement - self.robot.loc)
      move_to = move_to * .7 + self.move_to * .3
    
    
    point_dir = self.points[0] - self.robot.loc
    target_rot = -math.atan2(point_dir[1], point_dir[0])
    self.pid.set_target(move_to, target_rot)
    actions = self.pid.run()
    self.actions = actions
    self.move_to = move_to
    return actions
    
class striker(get_open):
  def __init__(self, ball_controller, goal, game):
    self.goal = goal
    self.ball_controller = ball_controller
    if ball_controller.is_blue:
      self.allies = game.blue_robots
      self.enemies = game.yellow_robots
    else:
      self.allies = game.yellow_robots
      self.enemies = game.blue_robots
    get_open.__init__(self, [ball_controller.loc, goal], [1.3, 0.4], self.enemies, self.allies)
  def add(self, robot, game):
    robot.task = 1
    get_open.add(self, robot, game)
  def run(self):
    self.points[0] = self.ball_controller.loc
    return get_open.run(self)
    
class fielder(get_open):
  def __init__(self, ball_controller, target_to_support, game):
    self.target_to_support = target_to_support
    self.ball_controller = ball_controller
    if ball_controller.is_blue:
      self.allies = game.blue_robots
      self.enemies = game.yellow_robots
    else:
      self.allies = game.yellow_robots
      self.enemies = game.blue_robots
    get_open.__init__(self, [target_to_support.loc, ball_controller.loc], [0.7, 1], self.enemies, self.allies)
  def add(self, robot, game):
    robot.task = 2
    get_open.add(self, robot, game)
  def run(self):
    self.points[0] = self.target_to_support.loc
    self.points[1] = self.ball_controller.loc
    return get_open.run(self)
    

if __name__ == "__main__":
  max_bots_per_team = 6
  game = PYsim(max_bots_per_team)
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  
  '''
  create the strikers
  '''
  for i in game.yellow_robots[1:3]:
    game.add_action(striker(game.yellow_robots[0], np.array([-5000,0]), game), i.id, False)
      
  '''
  Create the fielders
  '''
  for i in game.yellow_robots[3:5]:
    game.add_action(fielder(game.yellow_robots[0], game.yellow_robots[i.id - 2], game), i.id, False)
    
    
  while 1:
      
      
    '''
    handle resets and allow robots to be placed for debugging
    '''
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