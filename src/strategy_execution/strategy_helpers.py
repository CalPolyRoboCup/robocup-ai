import numpy as np
import math
import time
import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/..')
from basic_skills.source.move_to import *
from basic_skills.source.robot import *
from basic_skills.source.cover import *
from basic_skills.source.orbit_ball import *
from basic_skills.source.get_open import *
from basic_skills.source.Goalie import *
from basic_skills.source.Ball_Interception import *
from basic_skills.source.pass_to import *
from basic_skills.source.helper_functions import *

class team:
  def __init__(self, game, is_blue):
    self.is_blue = is_blue
    self.game = game
    if is_blue:
      self.allies = game.blue_robots
      #hack to identify goalie
      self.enemies = game.yellow_robots
      self.my_goal = np.array([-7000,0])
      self.their_goal = np.array([7000,0])
      self.strike_locations = [[3000, 1500], [3000, -1500]]
    else:
      self.enemies = game.blue_robots
      self.allies = game.yellow_robots
      self.my_goal = np.array([6000,0])
      self.their_goal = np.array([-6000,0])
      self.strike_locations = [[-3000, 1500], [-3000, -1500]]
    self.blocker_enemies = [robot(not is_blue, -1, None), robot(not is_blue, -1, None)]
    self.blocker_enemies[0].loc = self.their_goal + np.array([0, self.game.goal_height/2])
    self.blocker_enemies[1].loc = self.their_goal + np.array([0, -self.game.goal_height/2])
    self.blocker_enemies.extend(self.enemies)
    self.goalie = self.allies[-1]

    self.ball_controler = -1
    
    #this is a proxy so we can use pass_to to shoot at the goal
    self.enemy_goal_bot = robot(is_blue, -1, game)
    self.enemy_goal_bot.loc = self.their_goal
   
    self.goalie_action = Goalie()
    self.pass_action = pass_to(self.allies[0])
    self.dribble_action = get_open_with_ball([self.allies[0]], self.allies, self.blocker_enemies) #protect_ball()
    self.intercept_action = intercept_ball()
    self.striker_actions = [striker(None, self.their_goal, self.enemies, self.allies) for _ in range(2)]
    self.fielder_actions = [fielder(self.game.ball, None, self.enemies, self.allies) for _ in range(3)]
    
    
    self.prints = []


def get_closest(enemy, free_allies):
  best_dist = travel_distance(enemy, free_allies[0])
  best = free_allies[0]
  for fa in free_allies[1:]:
    dist = travel_distance(enemy, fa)
    if dist < best_dist:
      best = fa
      best_dist = dist
  return best
    
NUM_CELLS = 20
#get the cell that the loc is in see Strategy Documentation on gdrive page 2
'''
current grid numbers
1   6     11    16
2   7     12    17
3   8     13    18
4   9     14    19
5   10    15    20
'''

  

'''
goal area 
0    1000              8000 9000
2000
4000
'''

'''
dims
    2500    4500  6500
1000
2000
4000
5000
''' 
def get_cell(loc):
  loc = loc + np.array([4500, 3000])
  if loc[0] < 2500:
    cell = 1
  elif loc[0] < 4500:
    cell = 6
  elif loc[0] < 6500:
    cell = 11
  else:
    cell = 16
    
  
  if loc[1] < 1000:
    cell += 0
  elif loc[1] < 2000:
    cell += 1
  elif loc[1] < 4000:
    cell += 2
  elif loc[1] < 5000:
    cell += 3
  else:
    cell += 4
    
  return cell
  
  
def squash(value, limit):
  if value > limit: 
    return 1
  return value/limit

def assign_strikers_and_fielders(team, free_allies):
  ind = 0
  for i in free_allies[:2]:
    team.striker_actions[ind].ball_controller = team.allies[team.ball_controler]
    team.game.add_action(team.striker_actions[ind], i.id, False)
    ind += 1
    
      
  '''
  Create the fielders
  assumes yellow_robot 0 has the ball and yellow_robot i.id - 2 is a striker
  '''
  ind = 0
  for i in free_allies[2:]:
    print(ind, len(team.fielder_actions), team.ball_controler)
    team.fielder_actions[ind].ball_controler = team.allies[team.ball_controler]
    team.fielder_actions[ind].target_to_support = free_allies[ind]
    team.game.add_action(team.fielder_actions[ind], i.id, False)
    ind += 1