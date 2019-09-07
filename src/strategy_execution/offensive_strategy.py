import sys
import os
import numpy as np
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from state_machine import state
from strategy_helpers import assign_Strikers_and_Fielders
from evaluate_robots import evaluate_robots
from neutral_strategy import neutral_strategy
from defensive_strategy import defensive_strategy
from passing_strategy import passing_strategy
from strategy_numbers import *


sys.path.insert(0, dirname+'/..')
from basic_skills.source.worst_intercept import worst_intercept
from basic_skills.source.helper_functions import dist
from basic_skills.source.GetOpen import GetOpenForKick
from basic_skills.source.PassTo import KickTo


#assigns actions to robots when team has the ball
class offensive_strategy(state):
    state_number = 2
    def __init__(self, id, team):
        state.__init__(self, id)
        self.team = team
        
        self.pass_signal_linger = 10
        self.goal_shot_margin = 500
        self.best_reciever_index = 0

        self.dribble_action = None
        self.kick_action = KickTo(self.team.enemy_goal)
        
    def setup(self):
        #print("attack", self.team.is_blue)
        free_allies = [a for a in self.team.field_players]
        if self.team.ball_controler in free_allies:
            free_allies.remove(self.team.ball_controler)
        assign_Strikers_and_Fielders(self.team, free_allies)

        #get open for a striker and a fielder gives good defensive positioning
        self.dribble_action = GetOpenForKick([free_allies[0], free_allies[2]],
                                self.team.blocker_enemies, self.team.allies)

        self.team.game.add_action(self.dribble_action, self.team.ball_controler.id, self.team.is_blue)

    def update(self):
        if self.team.pass_action.done():
            # if our ball controler no longer controls the ball
            return PASSING_STRATEGY_STATE_NUMBER

        if self.team.ball_controler is not None and self.team.ball_controler.is_blue != self.team.is_blue:
            # if the enemy has the ball
            return DEFENSIVE_STRATEGY_STATE_NUMBER
        
        if self.team.ball_controler is None:
            # if we droped the ball
            return NEUTRAL_STRATEGY_STATE_NUMBER
            
        self.handle_ball()

        return OFFENSIVE_STRATEGY_STATE_NUMBER
            
    def handle_ball(self):
        # if we can shoot at the goal and haven't already started shooting
        goal_shot_safety, _ = worst_intercept(self.team.ball_controler.loc, self.team.enemy_goal, self.team.enemies)
        if goal_shot_safety is None or abs(goal_shot_safety) > self.goal_shot_margin:
            ##
            ##TODO: should replace this with dedicated goal shot code
            ##
            if self.team.ball_controler.action != self.kick_action:
                self.team.game.add_action(self.kick_action, self.team.ball_controler.id, self.team.is_blue)
            ##
            ##TODO: should replace this with dedicated goal shot code
            ##
            if self.kick_action.done():
                print("blue " if self.team.is_blue else "yellow", "team takes shot at goal")

        else:
            scores, self.best_reciever_index = evaluate_robots(self.team, self.best_reciever_index)
            best_score = scores[self.best_reciever_index]
            if best_score > scores[self.team.ball_controler.id] or self.team.ball_controler.id == self.team.goalie.id:
                self.team.pass_action.target_robot = self.team.allies[self.best_reciever_index]
                self.team.game.add_action(self.team.pass_action, self.team.ball_controler.id, self.team.is_blue)
            elif self.best_reciever_index != self.team.ball_controler.id:
                if self.dribble_action.support_robots[0].id != self.best_reciever_index:
                    self.dribble_action.support_robots[0] = self.team.allies[self.best_reciever_index]
                else:
                    self.dribble_action.support_robots[1] = self.team.allies[self.best_reciever_index]

                self.team.game.add_action(self.dribble_action, self.team.ball_controler.id, self.team.is_blue)