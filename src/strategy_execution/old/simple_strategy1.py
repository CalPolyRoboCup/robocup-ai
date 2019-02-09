import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/Users/nathan/Documents/robocup-ai/src')
from basic_skills.action import *
from basic_skills.move_to.move_to import *
from basic_skills.cover.cover import *
from basic_skills.ball_interception.Ball_Interception import *
from basic_skills.action_sequence import *
from basic_skills.helper_functions import *
from matplotlib.widgets import Slider, Button, RadioButtons
from pygame_simulator.PySim import *
from assign_closest import *
    
def cull_close(list):
  q = 0
  for l in list:
    p = q + 1
    while p < len(list):
      if np.linalg.norm(np.array(l) - np.array(list[p])) < 600:
        list.pop(p)
      p += 1
    q += 1
  return list

def get_interesting_points(allies, enemies):
  res = []
  i = 0
  for e in enemies:
    i += 1
    for ee in enemies[i:]:
      res.append((e.loc + ee.loc)/2)
  return res
  
def rate_positions(locations, enemies, important_static_points):
  res = []
  important_passes = []
  for l in locations:
    value = 5
    ips = []
    for goal in important_static_points:
      pass_value = rate_pass(l, goal, enemies)
      ips.append(pass_value)
      value += pass_value*5
    for e in enemies:
      if np.linalg.norm(l - e.loc) < 600:
        value -= 2
    res.append(value)
    important_passes.append(ips)
  
  passes = []
  for l in locations:
    ipass = []
    for ll in locations:
      ipass.append(rate_pass(l, ll, enemies))
    passes.append(ipass)
  
  for k in range(2):
    i = 0
    for l in locations:
      j = 0
      value = 0
      for ll in locations:
        if i != j:
          j += 1
          continue
        value += passes[i][j] * res[j] / 2
        j += 1
      res[i] += value
      i += 1
  return res, important_passes
    
def rate_pass(p1, p2, enemies):
  # value between 0 and 1 
  # 1 uncontested shot
  # 0 already blocked
  robot_speed = 1
  ball_speed = 1.5*robot_speed
  worst = -1
  pvec = np.array(p2) - np.array(p1)
  pangle = math.atan2(pvec[1], pvec[0])
  pmag = np.linalg.norm(pvec)
  for e in enemies:
    local = convert_local(e.loc - p1, pangle)
    if local[0] < 0:
      dist = np.linalg.norm(local)
      dist += robot_speed*dist/ball_speed
    elif local[0] > pmag:
      dist = np.linalg.norm(local - np.array([pmag, 0]))
      dist -= robot_speed*local[0]/ball_speed
    else:
      dist = local[1]
      dist -= robot_speed*local[0]/ball_speed
    if worst < 0 or dist < worst:
      worst = dist
  if worst < 0:
    worst = 0
  return worst/10000
    
class strategy:
  def __init__(self, game, is_blue):
    self.is_blue = is_blue
    self.game = game
    if is_blue:
      self.allies = game.blue_robots
      self.enemies = game.yellow_robots
      self.my_goal = [-5000,0]
      self.their_goal = [5000,0]
      self.always_useful = [[-1000, 2000], [-1000, 0], [-1000, -2000], [2000, 2000], [2000, 0], [2000, -2000]]
    else:
      self.enemies = game.blue_robots
      self.allies = game.yellow_robots
      self.my_goal = [5000,0]
      self.their_goal = [-5000,0]
      self.always_useful = [[1000, 2000], [1000, 0], [1000, -2000], [-2000, 2000], [-2000, 0], [-2000, -2000]]
    self.goalie = self.allies[-1]
    self.allies = self.allies[:-1]
    
    self.attacking = False
    self.defending = False
    self.passing = False
    
    self.pass_decay = 10
    
  def defensive_strategy(self, exclude = []):
    available_allies = []
    for i in range(len(self.allies)):
      if i not in exclude:
        available_allies.append(self.allies[i])
    if not self.defending:
      self.defending = True
      self.attacking = False
      enemy_locs = []
      farthest_back_ind = -1
      farthest_back_dist = 0 
      ind = 0
      for e in self.enemies:
        # We will treat the enemy with the ball differently
        if self.game.ball.controler == False or e.id != self.game.ball.controler.id:
          enemy_locs.append(e.loc)
        if self.is_blue and e.loc[0] > farthest_back_dist or farthest_back_ind == -1:
          farthest_back_ind = ind
          farthest_back_dist = e.loc[0]
        if (not self.is_blue) and e.loc[0] < farthest_back_dist or farthest_back_ind == -1:
          farthest_back_ind = ind
          farthest_back_dist = e.loc[0]
        ind += 1
      
      #we will ignore the goalie
      if len(enemy_locs) != farthest_back_ind:
        enemy_locs.pop(farthest_back_ind)
      
      assignments = assign_closest(enemy_locs, available_allies)
      ind = 0
      for a in available_allies:
        if ind not in assignments:
          # pressure main bot
          a.add_action(cover(self.my_goal, self.game.ball.controler, min_interpose_offset = 190))
        else:
          aind = assignments.index(ind)
          if aind >= farthest_back_ind:
            aind += 1
          enemy = self.enemies[aind]
          if not type(a.action) is cover_robots:
            if self.game.ball.controler != False:
              if aind >= self.game.ball.controler.id and self.game.ball.controler.id != farthest_back_ind:
                aind += 1
              enemy = self.enemies[aind]
              a.add_action(cover_robots(enemy, self.game.ball.controler, min_interpose_offset = 190))
            else:
              a.add_action(cover_robots(enemy, self.game.ball, lead_target_robot = False, min_interpose_offset = 190))
          else:
            a.action.target_bot1 = enemy
            a.action.target_bot2 = self.game.ball
          
        ind += 1
  def offensive_strategy(self, exclude = []):
    if not self.attacking:
      self.defending = False
      self.attacking = True
    interesting_locations = get_interesting_points(self.allies, self.enemies)
    interesting_locations.extend(self.always_useful)
    interesting_locations = cull_close(interesting_locations)
    values, _ = rate_positions(interesting_locations, self.enemies, [self.game.ball.loc, self.their_goal])
    il_v = zip(interesting_locations, values)
    best_spots = sorted(il_v, key = lambda val : -val[1])
    top_spots = [bs[0] for bs in best_spots]
    self.best_spots = best_spots
    
    free_allies = []
    for i in range(len(self.allies)):
      if self.game.ball.controler != False and i != self.game.ball.controler.id and i not in exclude:
        free_allies.append(self.allies[i])
      elif i not in exclude:
        free_allies.append(self.allies[i])
    assignments = assign_closest(top_spots, free_allies)
    ind = 0
    #print()
    #print(197, len(self.allies), len(free_allies))
    for a in assignments:
      if a == -1:
        break
      print(type(free_allies[a].action))
      if self.game.ball.controler != False:
        if not type(free_allies[a].action) is cover:
          free_allies[a].add_action(cover(top_spots[ind], self.game.ball.controler, interpose_factor = 1))
        else:
          free_allies[a].action.target_loc = top_spots[ind]
          free_allies[a].action.target_robot = self.game.ball.controler
      else:
        if not type(free_allies[a].action) is cover_ball:
          free_allies[a].add_action(cover_ball(top_spots[ind], self.game.ball, interpose_factor = 1))
        else:
          free_allies[a].action.target_loc = top_spots[ind]
          free_allies[a].action.target_ball = self.game.ball
        
      print(free_allies[a].id, best_spots[ind], free_allies[a].action.move_to, free_allies[a].action.interpose_factor)
      ind += 1
      
    pv = self.game.ball.loc - free_allies[assignments[0]].loc
    pv = 125 / np.linalg.norm(pv) * pv
    
    
    self.internal_ratings = [(free_allies[i].loc, values[i]) for i in range(len(free_allies))]
    
    good_pass_threshold = .2
    if self.game.ball.controler != False and self.game.ball.controler.is_blue == self.is_blue:
      #self.game.ball.controler.add_action(move_to(self.game.ball.loc + pv, math.atan2(pv[1], pv[0])))
      goal_shot = rate_pass(self.game.ball.loc, self.their_goal, self.enemies)
      if goal_shot > good_pass_threshold:
        print("shoot at goal", goal_shot, self.is_blue)
        pv = self.game.ball.controler.loc - self.their_goal
        pv_scaleed = pv * 125 / np.linalg.norm(pv)
        self.game.ball.loc = self.game.ball.controler.loc - pv_scaleed
        self.game.ball.velocity = -pv_scaleed*3
      else:
        controler_value = self.pass_decay
        values, passes = rate_positions([fa.loc for fa in free_allies], self.enemies, [self.game.ball.loc, self.their_goal])
          
        for i in range(len(passes)):
          #print(passes[i], good_pass_threshold, values[i], controler_value)
          if passes[i][0] > good_pass_threshold and values[i] > controler_value:
            print("pass to ", free_allies[i].id, self.is_blue, passes[i])
            pv = self.game.ball.loc - free_allies[i].loc
            pv_scaleed = pv * 125 / np.linalg.norm(pv)
            self.game.ball.loc = self.game.ball.controler.loc - pv_scaleed
            self.game.ball.velocity = -pv_scaleed*3
            #self.game.ball.controler.add_action(pass_to(free_allies[i]))
            self.pass_decay = 10
        self.pass_decay -= .1
    input()
    
  def neutral_strategy(self):
    closest = assign_closest([self.game.ball.loc], self.allies)[0]
    self.allies[closest].add_action(intercept_ball())
    self.offensive_strategy(exclude = [closest])
    
  def update(self):
    # technically we should recalculate this since this is a hidden simulation variable
    # TODO: fix that
    #print(self.game.ball.controller)
    if not type(self.goalie.action) is cover:
      self.goalie.add_action(cover_ball(self.my_goal, self.game.ball, interpose_factor = 1, min_interpose_offset = 400))
    # if self.game.ball.controler == False and not self.passing:
      # self.neutral_strategy()
    # elif self.game.ball.controler.is_blue == self.is_blue:
      # self.offensive_strategy()
    # else:
      # self.defensive_strategy()
    self.neutral_strategy()
    
if __name__ == "__main__":
  game = PYsim(6)
  
  blue_strategy = strategy(game, is_blue = True)
  yellow_strategy = strategy(game, is_blue = False)
  
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  while 1:
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
    new_time = clock.tick()
    if blue_strategy.attacking:
      kp = blue_strategy.internal_ratings
      kp.extend(blue_strategy.best_spots)
    elif yellow_strategy.attacking:
      kp = yellow_strategy.internal_ratings
      kp.extend(yellow_strategy.best_spots)
    else:
      kp = []
    game.step(key_points = kp)
    #blue_strategy.update()
    yellow_strategy.update()
    ttime = new_time