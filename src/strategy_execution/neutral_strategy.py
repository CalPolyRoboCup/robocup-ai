import sys
import os
import numpy as np
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from state_machine import state
from strategy_numbers import *

dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/..')
from basic_skills.source.InterceptBall import InterceptBall
from strategy_helpers import get_closest, assign_Strikers_and_Fielders

class neutral_strategy(state):
    # assigns actions to robots when neither team controls the ball
    state_number = 0
    def __init__(self, id, team):
        state.__init__(self, id)
        self.team = team
        self.closest = None
        self.lead_factor = 5

    def setup(self, closest = None):
        # the robot closest to the ball tries to get the ball
        if closest == None:
            print("neutral")
            ball_future_pos = self.team.game.ball.loc + self.team.game.ball.velocity * self.lead_factor
            closest = get_closest(ball_future_pos, self.team.allies)
        self.closest = closest.id
        self.team.game.add_action(self.team.intercept_action, closest.id, self.team.is_blue)

        free_allies = []
        for fa in free_allies:
            if fa.id != closest.id:
                free_allies.append(fa)
                break
                
        # other robots spread out
        assign_Strikers_and_Fielders(self.team, free_allies)

    def update(self):
        ball_future_pos = self.team.game.ball.loc + self.team.game.ball.velocity * self.lead_factor
        closest = get_closest(ball_future_pos, self.team.allies)
        if closest.id != self.closest:
            self.setup(closest)

        # if someone has gotten the ball
        if self.team.ball_controler != -1:
            if self.team.ball_controler.is_blue == self.team.is_blue:
                return OFFENSIVE_STRATEGY_STATE_NUMBER
            else:
                return DEFENSIVE_STRATEGY_STATE_NUMBER
        return NEUTRAL_STRATEGY_STATE_NUMBER