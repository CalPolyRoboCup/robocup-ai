import numpy as np
import math
import time
import sys
import os

dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/..')
from worst_intercept import worst_intercept
from strategy_helpers import *   
  
  
'''
brief - runs a heuristic to determine the advantage of making a pass to each robot on the team
        HEURISTIC COMPNONENTS
          sticky_value - constant given to the robot with the highest score on the last 
                         cycle (adding a facing component to the passability metric probably 
                         makes this irrelevant)
          shot_value - weighting for ability to make a shot on the enemy goal
          intercept_clip - maximum interception range of an enemy
          value_propagation_rate - all robots gain value proportional to this factor times
                                   the value of other robots it can pass to (the can pass
                                   metric is non binary)
          enemy fear_value - penalizes positions that are close to an enemy even if the enemy
                             isn't blocking key shots
          down_field_value - value derived from being close to the enemy goal
          controler_value - value given to the robot that controls the ball. Ment to serve as
                            a threshold for the shot
params - team - team object. Specifies allies to evaluate and enemies to consider as blockers
         best_reciever_index - the index of the best robot to pass to in the last time step
                               used for sticky value
returns - values : heuristic estimates of the value of a pass to each robot in team 
          best_reciever_index : index of the robot with the highest score (in above vector)
                                except the ball_controler
'''
def evaluate_robots(team, best_reciever_index):
  #stime = time.time()

  sticky_value = 30
  shot_value = 500
  intercept_clip = 200
  value_propagation_rate = 0.3
  inate_value = 200
  enemy_fear_value = 600
  enemy_fear_radius = 1000
  down_field_value_weight = 0.001
  controler_value = 40

  #temp support for cell values
  cell_values = [0,10,0,10,0,20,40,60,40,20,10,20,30,20,10,5,10,15,10,5]   

  raw_values = []
  for r in team.allies:        
    worst, _, _ = worst_intercept(r.loc, team.their_goal, team.blocker_enemies + [e.loc for e in team.enemies])
    shot_likelyhood = squash(worst, intercept_clip)
    cell_value = cell_values[get_cell(r.loc) - 1]
    down_field_value = r.loc[0] * down_field_value_weight * (1 if team.is_blue else -1)
    sticky_value = sticky_value if (r.id == best_reciever_index) else 0
    fear_value = 0
    for e in team.enemies:
      dist = np.linalg.norm(e.loc - r.loc)
      fear_value = enemy_fear_value * (1 - squash(dist, enemy_fear_radius))
      
    if r.id == team.ball_controler:
      controler_value = controler_value
    else:
      controler_value = 0
      
    raw_values.append(shot_likelyhood * shot_value + cell_value + sticky_value + down_field_value + inate_value + controler_value - fear_value)
   
  highest_value_reciever = -1000
  values = [v for v in raw_values]
  for r in team.allies:
    worst, _, _ = worst_intercept(team.game.ball.loc, r.loc, team.blocker_enemies + [e.loc for e in team.enemies])
    pass_likelyhood = squash(worst, intercept_clip)
    for other in team.allies:
      if other.id != r.id:
        worst, _, _ = worst_intercept(r.loc, other.loc, team.blocker_enemies)
        secondary_pass_likelyhood = squash(worst, intercept_clip)
        values[r.id] += value_propagation_rate * raw_values[other.id] * secondary_pass_likelyhood
        
    values[r.id] *= pass_likelyhood
    
    if highest_value_reciever < values[r.id] and r.id != team.ball_controler:
      highest_value_reciever = values[r.id]
      best_reciever_index = r.id
  
  #update prints for debugging
  '''print(raw_values)
  print(values)
  print()'''
  team.prints = []
  for v, a in zip(values, team.allies):
    if v == max(values):
      team.prints.append((a.loc, -v/100));
    else:
      team.prints.append((a.loc, v/100));
  #print(time.time() - stime)
  return values, best_reciever_index