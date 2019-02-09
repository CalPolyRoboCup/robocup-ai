import numpy as np
import math
import sys
sys.path.insert(0, '../..')
from basic_skills.action import *
from basic_skills.get_open.get_open import *
from basic_skills.pass_to.pass_to import pass_to
from basic_skills.helper_functions import min_angle, travel_distance

from pygame_simulator.PySim_noise import *

class handle_ball(action):
  '''
  brief: Top level logic for the robot that currently has the ball.
        Decides who to pass to or rather to dribble the ball to a better location
  '''
  def __init__(self, allies, enemies, goal):
    '''
    params: allies - your allies
            enemies - your enemies
    '''
    action.__init__(self)
    
    # score representing how well this robot can make passes to points
    self.current_score = 0
    
    self.allies = allies
    self.enemies = enemies
    self.goal = goal
    
    self.goal_shot_margin = 500
    self.task_weights = {1:2.0, 2:0.1, 3:0.0}
    
    self.pass_signal_linger = 10
    self.passing = 0
    self.last_best = 0
    
    self.dribble = get_open_with_ball([allies[0]], allies, enemies)
    self.pass_to = pass_to()
  def add(self, robot, game):
    robot.task = 4
    action.add(self, robot, game)
    get_open_with_ball.add(self.dribble, robot, game)
    pass_to.add(self.pass_to, robot, game)
    
  def calculate_self_value(self):
    '''
    this should be elaborated on
    '''
    score = 850
    return score
    
  def run(self):
    best = self.allies[0]
    best_score = 0
    first = True
    ind = 0
    for a in self.allies:
      if issubclass(type(a.action), get_open) and a.id != self.robot.id:
        score = a.action.current_score * self.task_weights[a.task]
        worst = worst_intercept(self.robot.loc, a.loc, self.enemies)[0]
        score = score * worst / 500
        if self.last_best == a.id:
          score += 20
        if first or score > best_score:
          first = False
          best = a
          best_score = score
      ind += 1
    
    if worst_intercept(self.robot.loc, self.goal, self.enemies)[0] > self.goal_shot_margin:
      best = robot(False, -1, None)
      best.loc = self.goal
    
    self.current_score = self.calculate_self_value()
    
    if best_score > self.current_score:
      self.pass_to.target_robot = best
      actions = self.pass_to.run()
    else:
      '''
      
      I don't know what this was doing, but it seems to work without it
      
      run_away_location = self.get_run_away_location()
      self.dribble.target_loc = run_away_location'''
      actions = self.dribble.run()
    if (actions[0]):
      self.passing = self.pass_signal_linger
    elif self.passing:
      self.passing -= 1
    self.last_best = best.id
    return actions
    

if __name__ == "__main__":
  max_bots_per_team = 6
  game = PYsim(max_bots_per_team)
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  
  
  '''
  Create ball controller
  assumes yellow_robot 1 has the highest pass values and yellow_robot 0 has the ball or can easily get the ball
  '''
  game.add_action(handle_ball(game.yellow_robots, game.blue_robots, [-7000, 0]), 0, False)
  
  
  '''
  Create the strikers
  assumes yellow_robot 0 has the ball and [-5000,0] is the goal
  '''
  for i in game.yellow_robots[1:3]:
    game.add_action(striker(game.yellow_robots[0], np.array([-5000,0]), game.blue_robots, game.yellow_robots), i.id, False)
      
  '''
  Create the fielders
  assumes yellow_robot 0 has the ball and yellow_robot i.id - 2 is a striker
  '''
  for i in game.yellow_robots[3:]:
    game.add_action(fielder(game.yellow_robots[0], game.yellow_robots[i.id - 2], game.blue_robots, game.yellow_robots), i.id, False)
    
    
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
    game.step(key_points = [(y.loc, y.action.current_score/50*game.yellow_robots[0].action.task_weights[y.task]) for y in game.yellow_robots])
    ttime = new_time