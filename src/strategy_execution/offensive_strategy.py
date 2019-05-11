import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from state_machine import state
from strategy_helpers import assign_strikers_and_fielders

import numpy as np
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/..')
from worst_intercept import worst_intercept
from evaluate_robots import evaluate_robots

#assigns actions to robots when team has the ball
class offensive_strategy(state): 
  def __init__(self, id, neutral_id, defensive_id, passing_id, team):
    state.__init__(self, id)
    self.passing_id = passing_id
    self.neutral_id = neutral_id
    self.defensive_id = defensive_id
    self.team = team
    
    self.pass_signal_linger = 10
    
    self.goal_shot_margin = 100
    self.best_reciever_index = 0
    
  def setup(self):
    print("attack")
    self.transition = False
    free_allies = [a for a in self.team.allies]
    for fa in free_allies:
      if fa.id == self.team.ball_controler:
        free_allies.remove(fa)
        break
    assign_strikers_and_fielders(self.team, free_allies)
  def update(self):
    #put the handle_ball action on the ball_controler
    #ball_controler must be set to the id of the robot to control the ball by the
    #   state that transitioned to offense
    #
    #all other robots become strikers and fielders
      
      #chooses what the ball controler should do
    self.handle_ball()
        
    #if our ball controller has passed
    if self.team.pass_action.done():
      return self.passing_id
      
    #if the enemy has the ball
    if self.team.game.ball.controler != False and self.team.game.ball.controler.is_blue != self.team.is_blue:
      self.team.ball_controler = self.team.game.ball.controler.id
      return self.defensive_id
      
    #if our ball controler no longer controls the ball
    elif np.linalg.norm(self.team.allies[self.team.ball_controler].loc - self.team.game.ball.loc) > 500:
      return self.neutral_id
      
    return self.id
      
  def handle_ball(self):
    # if we can shoot at the goal and haven't already started shooting
    goal_shot_safety = worst_intercept(self.team.allies[self.team.ball_controler].loc, self.team.their_goal, self.team.enemies)[0]
    if goal_shot_safety > self.goal_shot_margin:
      ##
      ##should replace this with dedicated goal shot code
      ##
      self.team.pass_action.target_robot = self.team.enemy_goal_bot
      self.team.game.add_action(self.team.pass_action, self.team.ball_controler, self.team.is_blue)
    else:
      scores, self.best_reciever_index = evaluate_robots(self.team, self.best_reciever_index)
      best_score = max(scores)
      best_index = scores.index(best_score)
      if best_score > scores[self.team.ball_controler]:
        self.team.pass_action.target_robot = self.team.allies[best_index]
        self.team.game.add_action(self.team.pass_action, self.team.ball_controler, self.team.is_blue)
      else:
        self.team.dribble_action.support_robots = [self.team.allies[self.best_reciever_index]]
        self.team.game.add_action(self.team.dribble_action, self.team.ball_controler, self.team.is_blue)