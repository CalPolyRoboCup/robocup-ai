import numpy as np
import math
import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '..')
from basic_skills.action import *
from basic_skills.move_to.move_to import *
from basic_skills.cover.cover import *
from basic_skills.orbit_ball.orbit_ball import *
from basic_skills.get_open.get_open import *
from basic_skills.Goalie.Goalie import *
from basic_skills.handle_ball.handle_ball import *
from basic_skills.ball_interception.Ball_Interception import *
from basic_skills.action_sequence import *
from basic_skills.helper_functions import *
from matplotlib.widgets import Slider, Button, RadioButtons
from pygame_simulator.PySim_noise import *
    
def get_closest(enemy, free_allies):
  best_dist = travel_distance(enemy, free_allies[0])
  best = free_allies[0]
  for fa in free_allies[1:]:
    dist = travel_distance(enemy, fa)
    if dist < best_dist:
      best = fa
      best_dist = dist
  return best
    
class strategy:
  def __init__(self, game, is_blue):
    self.is_blue = is_blue
    self.game = game
    if is_blue:
      self.allies = game.blue_robots
      #hack to identify goalie
      self.enemies = game.yellow_robots
      self.my_goal = np.array([-5000,0])
      self.their_goal = np.array([5000,0])
    else:
      self.enemies = game.blue_robots
      self.allies = game.yellow_robots
      self.my_goal = np.array([5000,0])
      self.their_goal = np.array([-5000,0])
    self.blocker_enemies = [robot(not is_blue, -1, None), robot(not is_blue, -1, None)]
    self.blocker_enemies[0].loc = self.their_goal + np.array([0, self.game.goal_height/2])
    self.blocker_enemies[1].loc = self.their_goal + np.array([0, -self.game.goal_height/2])
    self.goalie = self.allies[-1]
    self.internal_ratings = []
    
    
    self.neutral = True
    self.attacking = False
    self.defending = False
    self.passing = False
    self.transition = True
    self.pass_timer = 0
    self.ball_controler = -1
    
    self.pass_action = pass_to(self.allies[0])
    self.dribble_action = protect_ball()
    
    self.attempt_pass = 0
    self.pass_signal_linger = 10
    
  def defensive_strategy(self):
    #on entering state assign each robot an enemy to guard
    if self.transition:
      print("defend")
      self.transition = False
      free_allies = [a for a in self.allies]
      open_enemies = [e for e in self.enemies]
      
      closest = get_closest(self.enemies[self.ball_controler], free_allies)
      self.game.add_action(cover(self.my_goal, self.enemies[self.ball_controler], min_interpose_offset = 190), closest.id, self.is_blue)
      
      for oe in open_enemies:
        if self.enemies[self.ball_controler] == oe.id:
          open_enemies.remove(oe)
      for fa in free_allies:
        if closest.id == fa.id:
          free_allies.remove(fa)
      
      for e in open_enemies:
        if len(free_allies) == 0:
          break
        closest = get_closest(e, free_allies)
        self.game.add_action(cover_robots(e, self.enemies[self.ball_controler], min_interpose_offset = 190), closest.id, self.is_blue)
        for fa in free_allies:
          if closest.id == fa.id:
            free_allies.remove(fa)
            break
            
    #state machine transition 
    if self.game.ball.controler != False and self.game.ball.controler.is_blue == self.is_blue:
      self.ball_controler = self.game.ball.controler.id
      self.attacking = True
      self.defending = False
      self.transition = True
      #print("to offense")
      
    elif np.linalg.norm(self.enemies[self.ball_controler].loc - self.game.ball.loc) > 500:
      self.defending = False
      self.neutral = True
      self.transition = True
      #print("to neutral")
            
            
  def passing_strategy(self):
    #on entering state assign the robot to receive pass to intercept_ball and the robot that just passed to fielder
    if self.transition:
      print("pass to", self.handle_action.pass_to.target_robot.id)
      self.transition = False
      
      intercept_action = intercept_ball()
      self.game.add_action(intercept_action, self.handle_action.pass_to.target_robot.id, self.is_blue)
      self.game.add_action(fielder(self.game.ball, self.handle_action.pass_to.target_robot, self.enemies, self.allies), self.ball_controler, self.is_blue)
      
    #state machine transitions
    if self.game.ball.controler != False:
      #print("passing transition")
      self.transition = True
      
      self.ball_controler = self.game.ball.controler.id
      #if they have the ball under a robots control
      if self.game.ball.controler.is_blue != self.is_blue:
        self.passing = False
        self.defending = True
        
      #if we have the ball under a robots control
      if self.game.ball.controler.is_blue == self.is_blue:
        self.passing = False
        self.attacking = True
      
  def evaluate_robots(self):
    def squash_intercept(intercept):
      if intercept > 300:
        intercept = 300
      return intercept/300
  
    raw_values = []
    for r in self.allies:
      if r.id == self.ball_controler:
        raw_values.append(self.controler_value)
      else:
        shot_likelyhood = squash_intercept(worst_intercept(r.loc, self.their_goal, self.blocker_enemies + [e.loc for e in self.enemies]))
        cell_value = self.cell_values[get_cell(r.loc)]
        
        sticky_value = self.sticky_value if r.id == old_best_id else 0
        raw_values.append(shot_likelyhood * self.shot_value + cell_value + sticky_value)
     
    values = [v for v in raw_values]
    for r in self.allies:
      pass_likelyhood = squash_intercept(worst_intercept(self.game.ball.loc, r.loc, self.blocker_enemies + [e.loc for e in self.enemies]))
      for other in self.allies:
        if other.id != r.id:
          secondary_pass_likelyhood = squash_intercept(worst_intercept(r.loc, other.loc, self.blocker_enemies))
          values[r.id] += self.value_propagation_rate * raw_values[other.id] * secondary_pass_likelyhood
          
      values[r.id] *= pass_likelyhood
    
    return values
    
  def handle_ball(self):
    # if we can shoot at the goal and haven't already started shooting
    if worst_intercept(self.allies[self.ball_controler].loc, self.their_goal, self.enemies)[0] > self.goal_shot_margin:
      self.game.add_action(shoot_goal(self.is_blue), self.ball_controler, self.is_blue)
    else:
      scores = self.evaluate_robots()
      best_score = max(sores)
      best_index = scores.index(best_score)
      if best_score > scores[self.ball_controler]:
        self.pass_action.target_robot = self.allies[best_index]
        self.game.add_action(self.pass_action, self.ball_controler, self.is_blue)
        if (self.pass_action.kick):
          self.attempt_pass = self.pass_signal_linger
          self.pass_action.kick = 0
      else:
        self.game.add_action(self.dribble_action, self.ball_controler, self.is_blue)
        
    if self.attempt_pass:
      self.attempt_pass -= 1
    
  def offensive_strategy(self):
    #put the handle_ball action on the ball_controler
    #ball_controler must be set to the id of the robot to control the ball by the
    #   state that transitioned to offense
    #
    #all other robots become strikers and fielders
    if self.transition:
      print("attack")
      self.transition = False
      free_allies = [a for a in self.allies]
      self.game.add_action(self.handle_action, self.ball_controler, self.is_blue)
      for fa in free_allies:
        if fa.id == self.ball_controler:
          free_allies.remove(fa)
          break
      self.assign_strikers_and_fielders(free_allies)
        
        
    #if the ball controller wants to pass and the ball has started moving
    if self.attempt_pass and np.linalg.norm(self.game.ball.velocity) > 50 and self.game.ball.controler == False:
      self.attacking = False
      self.passing = True
      self.transition = True
      self.handle_action.passing = 0
      
    #if the enemy has the ball
    if self.game.ball.controler != False and self.game.ball.controler.is_blue != self.is_blue:
      self.attacking = False
      self.defending = True
      self.transition = True
      self.ball_controler = self.game.ball.controler.id
      
    elif np.linalg.norm(self.allies[self.ball_controler].loc - self.game.ball.loc) > 500:
      self.attacking = False
      self.neutral = True
      self.transition = True
      
      
  def neutral_strategy(self):
    closest = get_closest(self.game.ball, self.allies)
    if self.transition or closest.id != self.ball_controler:
      self.ball_controler = closest.id
      self.transition = False 
      print("neutral")
      free_allies = [a for a in self.allies]
      self.game.add_action(intercept_ball(), closest.id, self.is_blue)
      for fa in free_allies:
        if fa.id == closest.id:
          free_allies.remove(fa)
          break
      self.assign_strikers_and_fielders(free_allies)
    
    #if someone has gotten the ball
    if self.game.ball.controler != False:
      self.neutral = False
      self.transition = True
      self.ball_controler = self.game.ball.controler.id
      if self.game.ball.controler.is_blue == self.is_blue:
        self.attacking = True
      else:
        self.defending = True
        
  #grabs robots at random to fill outfield roles
  def assign_strikers_and_fielders(self, free_allies):
  
    for i in free_allies[:2]:
      game.add_action(striker(self.allies[self.ball_controler], self.their_goal, self.enemies, self.allies), i.id, False)
        
    '''
    Create the fielders
    assumes yellow_robot 0 has the ball and yellow_robot i.id - 2 is a striker
    '''
    ind = 0
    for i in free_allies[2:]:
      game.add_action(fielder(self.allies[self.ball_controler], free_allies[ind], self.enemies, self.allies), i.id, False)
      ind += 1
      
  def update(self):
    # technically we should recalculate self.game.ball.controller since it is a hidden simulation variable
    # TODO: fix that
    
    if not type(self.goalie.action) is Goalie:
      self.game.add_action(Goalie(), self.goalie.id, self.is_blue)
      
    
      
    if self.defending:
      self.defensive_strategy()
    elif self.attacking:
      self.offensive_strategy()
    elif self.passing:
      self.passing_strategy()
    else:
      self.neutral_strategy() 
      
  def reset(self):
    self.neutral = True
    self.attacking = False
    self.defending = False
    self.passing = False
    self.transition = True
      
if __name__ == "__main__":
  game = PYsim(6)
  
  blue_strategy = strategy(game, is_blue = True)
  yellow_strategy = strategy(game , is_blue = False)
  
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
          yellow_strategy.reset()
      if event.type == MOUSEBUTTONDOWN:
        pressed1, pressed2, pressed3 = pygame.mouse.get_pressed()
        #left mouse button
        if pressed1:
          print("high")
          game.yellow_robots_internal[1].loc = game.convert_to_field_position(pygame.mouse.get_pos())
          
    game.step()
    
    #my offensive_strategy currently isn't good enough to deal with my defensive_strategy
    #blue_strategy.update()
    
    
    yellow_strategy.update()