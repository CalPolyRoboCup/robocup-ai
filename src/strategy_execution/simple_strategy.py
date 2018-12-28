import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/Users/nathan/Documents/robocup-ai/src')
from basic_skills.action import *
from basic_skills.move_to.move_to import *
from basic_skills.cover.cover import *
from basic_skills.orbit_ball.orbit_ball import *
from basic_skills.get_open.get_open5b import *
from basic_skills.ball_interception.Ball_Interception2 import *
from basic_skills.action_sequence import *
from basic_skills.helper_functions import *
from matplotlib.widgets import Slider, Button, RadioButtons
from pygame_simulator.PySim_noise import *
from assign_closest import *
  
class sudo_kick(action):
  def run(self):
    return [1,0,0,0,0]
  
def rate_positions(locations, enemies, important_static_points):
  res = []
  important_passes = []
  for l in locations:
    value = 5
    ips = []
    for goal in important_static_points:
      pass_value = rate_pass(l, goal, enemies)
      ips.append(pass_value)
      value += pass_value*10
    for e in enemies:
      if np.linalg.norm(l - e.loc) < 750:
        value -= 2
    res.append(value)
    important_passes.append(ips)
  
  passes = []
  for l in locations:
    ipass = []
    for ll in locations:
      pass_value = rate_pass(l, ll, enemies)
      ipass.append(pass_value)
    passes.append(ipass)
  
  for k in range(1):
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
  return res
    
def rate_pass(p1, p2, enemies):
  # value between 0 and 1 
  # 1 uncontested shot
  # 0 already blocked
  ball_to_robot_speed = 10
  worst = -1
  first = True
  worst_local = np.array([0,0])
  pvec = p1 - p2
  pangle = -math.atan2(pvec[1], pvec[0])
  pmag = np.linalg.norm(pvec)
  for e in enemies:
    local = convert_local(e.loc - p2, -pangle)
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
  
  safety = 300
  
  if worst > safety:
    worst = safety
  worst = worst/safety
  return worst
    
class strategy:
  def __init__(self, game, is_blue):
    self.is_blue = is_blue
    self.game = game
    if is_blue:
      self.allies = game.blue_robots
      self.enemies = game.yellow_robots
      self.my_goal = [-6000,0]
      self.their_goal = [6000,0]
      self.always_useful = [[-1000, 2000], [-1000, 0], [-1000, -2000], [2000, 2000], [2000, 0], [2000, -2000]]
    else:
      self.enemies = game.blue_robots
      self.allies = game.yellow_robots
      self.my_goal = [6000,0]
      self.their_goal = [-6000,0]
      self.always_useful = [[1000, 2000], [1000, 0], [1000, -2000], [-2000, 2000], [-2000, 0], [-2000, -2000]]
    self.goalie = self.allies[-1]
    self.allies = self.allies[:-1]
    self.internal_ratings = []
    
    self.pass_wait = 240
    
    self.attacking = False
    self.defending = False
    self.neutral = False
    self.passing = 0
    self.pass_reciever = False
    
    self.pass_decay = 10
    
  def defensive_strategy(self, exclude = []):
    available_allies = []
    for i in range(len(self.allies)):
      if i not in exclude:
        available_allies.append(self.allies[i])
    if not self.defending:
      self.defending = True
      self.attacking = False
      self.neutral = False
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
          self.game.add_action(cover(self.my_goal, self.game.ball.controler, min_interpose_offset = 190), a.id, self.is_blue)
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
              self.game.add_action(cover_robots(enemy, self.game.ball.controler, min_interpose_offset = 190), a.id, self.is_blue)
            else:
              self.game.add_action(cover_robots(enemy, self.game.ball, lead_target_robot = False, min_interpose_offset = 190), a.id, self.is_blue)
          else:
            a.action.target_bot1 = enemy
            a.action.target_bot2 = self.game.ball
          
        ind += 1
  def offensive_strategy(self, exclude = []):
    #print("attack", self.is_blue)
    closest = -1
    if self.game.ball.controler == False:
      closest = assign_closest([self.game.ball.loc], self.allies)[0]
      self.game.add_action(intercept_ball(), closest, self.is_blue)
    free_allies = []
    for i in range(len(self.allies)):
      if (self.game.ball.controler == False or self.allies[i].id != self.game.ball.controler.id) and i not in exclude and (self.passing == 0 or self.allies[i].id != self.pass_reciever.id) and i != closest:
        free_allies.append(self.allies[i])
        
    #print(self.passing, "passing")
        
    if self.passing == 0:
      
      if not self.attacking:
        #print("first", self.is_blue)
        self.defending = False
        self.attacking = True
        self.neutral = False
      #two "strikers" set up to make shots
      for fa in free_allies[:-2]:
        if not type(fa.action) is get_open:
          self.game.add_action(get_open([self.game.ball.loc, self.their_goal],[1,0.5], self.enemies, self.allies), fa.id, self.is_blue)
        else:
          fa.action.points = [self.game.ball.loc, self.their_goal]
        fa.role = 0
        
      #two "fielders" set up double play shots
      i = 0
      for fa in free_allies[-2:]:
        if not type(fa.action) is get_open:
          self.game.add_action(get_open([self.game.ball.loc, free_allies[i].loc],[1,.7], self.enemies, self.allies), fa.id, self.is_blue)
        else:
          fa.action.points = [self.game.ball.loc, free_allies[i].loc]
        i += 1
        fa.role = i
    
    
      
    else:
      #print("p", end = "")
      self.passing -= 1
      #self.pass_reciever.add_action(intercept_ball())
      if self.game.ball.controler != False and self.passing < self.pass_wait - 3:
        #print("reset")
        self.passing = 0
    
    
    good_pass_threshold = .15
    #print()
    #if we have the ball
    if self.game.ball.controler != False and self.game.ball.controler.is_blue == self.is_blue:
      #print("we have the ball")
      goal_shot = rate_pass(self.game.ball.loc, self.their_goal, self.enemies)
      best_pass_loc = None
      best_value = 0
      first = True
      kicking = False
      if goal_shot > good_pass_threshold:
        best_pass_loc = self.their_goal
        shot_vec = self.their_goal - self.game.ball.controler.loc
        #print(normalize_angle(self.game.ball.controler.rot), -math.atan2(shot_vec[1], shot_vec[0]), abs(normalize_angle(self.game.ball.controler.rot + math.atan2(shot_vec[1], shot_vec[0]))), abs(normalize_angle(self.game.ball.controler.rot + math.atan2(shot_vec[1], shot_vec[0]))) < .01)
        if abs(normalize_angle(self.game.ball.controler.rot + math.atan2(shot_vec[1], shot_vec[0]))) < .02:
          pv = self.their_goal - self.game.ball.controler.loc
          pv_scaleed = pv * 125 / np.linalg.norm(pv)
          #self.game.ball.loc = self.game.ball.controler.loc + pv_scaleed
          
          print("shoot at goal", goal_shot, self.is_blue, self.game.ball.velocity)
          self.game.ball_internal.velocity = pv_scaleed*11
          self.game.add_action(get_open([self.game.ball.loc, self.their_goal], [1,0.5], self.enemies, self.allies), self.game.ball.controler.id, self.is_blue)
          self.game.ball.controler.task = 0
          self.passing = 240
          self.game.add_action(sudo_kick(), self.game.ball.controler.id, self.is_blue)
          kicking = True
          self.pass_decay = 10
      else:
        controler_value = self.pass_decay + 4
        for e in self.enemies:
          if np.linalg.norm(e.loc - self.game.ball.loc) < 750:
            controler_value -= 10
        values = rate_positions([fa.loc for fa in free_allies], self.enemies, [self.their_goal])
        self.internal_ratings = [(self.game.ball.controler.loc, controler_value)]
          
        for i in range(len(free_allies)):
          pass_rating = rate_pass(self.game.ball.loc, free_allies[i].loc, self.enemies)
          self.internal_ratings.append((free_allies[i].loc, values[i]*pass_rating))
          #print(pass_rating, good_pass_threshold, values[i] * pass_rating, controler_value, )
          if first or (pass_rating > good_pass_threshold and values[i] * pass_rating > best_value):
            best_pass_loc = free_allies[i].loc
            best_value = values[i] * pass_rating
            best_pass_rating = pass_rating
            best_allie = free_allies[i]
            first = False
            best_pass_vec = free_allies[i].loc - self.game.ball.controler.loc
          #print(free_allies[i].id, pass_rating, values[i] * pass_rating)
        #print(self.game.ball.controler.rot, math.atan2(best_pass_vec[1], best_pass_vec[0])) 
        if self.pass_decay > 0:
          self.pass_decay -= .1
        #print(best_pass_rating > good_pass_threshold, best_value > controler_value, abs(normalize_angle(self.game.ball.controler.rot - math.atan2(best_pass_vec[1], best_pass_vec[0]))) < .1)
        if best_pass_rating > good_pass_threshold and best_value > controler_value and abs(normalize_angle(self.game.ball.controler.rot + math.atan2(best_pass_vec[1], best_pass_vec[0]))) < .02:
          print("pass to ", free_allies[i].id, self.is_blue, pass_rating)
          #pv = best_pass_loc - self.game.ball.controler.loc
          #pv_scaleed = pv * 125 / np.linalg.norm(pv)
          #self.game.ball.loc = self.game.ball.controler.loc + pv_scaleed
          #self.game.ball_internal.velocity = pv_scaleed*11
          #self.game.ball.controler.add_action(pass_to(free_allies[i]))
          self.pass_decay = 10
          self.passing = self.pass_wait
          self.pass_reciever = best_allie
          #print(best_allie.id, self.game, self.game.add_action, intercept_ball(), self.is_blue)
          self.game.add_action(intercept_ball(), best_allie.id, self.is_blue)
          best_allie.action.iterations = 0
          self.game.add_action(sudo_kick(), self.game.ball.controler.id, self.is_blue)
          kicking = True
      if not kicking:
        self.game.add_action(orbit_ball(best_pass_loc), self.game.ball.controler.id, self.is_blue)
      
    
  def neutral_strategy(self):
    print("I am indifferent to this situation", self.neutral)
    if not self.neutral:
      self.neutral = True
      self.attacking = False
      self.defending = False
      closest = assign_closest([self.game.ball.loc], self.allies)[0]
      print("closest is ", closest, self.allies[closest].loc)
      self.game.add_action(intercept_ball(), closest, self.is_blue)
      free_allies = []
      for i in range(len(self.allies)):
        if i != closest:
          free_allies.append(self.allies[i])
      
      enemy_locs = []
      farthest_back_ind = -1
      farthest_back_dist = 0 
      ind = 0
      for e in self.enemies:
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
          
      assignments = assign_closest(enemy_locs, free_allies)
      ind = 0
      for i in range(len(assignments)):
        if ind == farthest_back_ind:
          ind == 1
        self.game.add_action(cover_ball(self.my_goal, self.enemies[ind], interpose_factor = .3), free_allies[assignments[i]].id, self.is_blue)
        ind += 1
    
  def update(self):
    # technically we should recalculate this since this is a hidden simulation variable
    # TODO: fix that
    #print(self.game.ball.controller)
    if not type(self.goalie.action) is cover:
      self.game.add_action(cover_ball(self.my_goal, self.game.ball, interpose_factor = 1, min_interpose_offset = 1000), self.goalie.id, self.is_blue)
    #if self.game.ball.controler == False and self.passing == 0:
      #self.neutral_strategy()
    if self.game.ball.controler != False and self.game.ball.controler.is_blue != self.is_blue:
      self.defensive_strategy()
    else:
      self.offensive_strategy()
    #self.offensive_strategy()
      
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
      if event.type == KEYDOWN or event.type == KEYUP:
        keys = pygame.key.get_pressed()
        #press r-key to reset
        if keys[K_r]:
          game.reset()
    new_time = clock.tick()
    if blue_strategy.attacking:
      kp = blue_strategy.internal_ratings
    elif yellow_strategy.attacking:
      kp = yellow_strategy.internal_ratings
    else:
      kp = []
    game.step()#key_points = kp
    blue_strategy.update()
    yellow_strategy.update()
    ttime = new_time