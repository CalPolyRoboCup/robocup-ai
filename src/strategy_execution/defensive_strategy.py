import sys
import os
import numpy as np
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from state_machine import state
from strategy_helpers import get_closest

dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from strategy_numbers import *

sys.path.insert(0, dirname+'/..')
from basic_skills.source.cover import cover
from basic_skills.source.helper_functions import mag

class defensive_strategy(state):
    # assigns actions to robots when the enemy has the ball  
    state_number = 1
    def __init__(self, id, team):
        state.__init__(self, id)
        self.team = team

    def setup(self):
        print("defend")
        free_allies = [a for a in self.team.field_players]
        open_enemies = [e for e in self.team.enemies]
        
        closest = get_closest(self.team.ball_controler, free_allies)
        cover_action = cover(self.team.my_goal, self.team.ball_controler)
        self.team.game.add_action(cover_action, closest.id, self.team.is_blue)
        
        open_enemies.pop(self.team.ball_controler.id)
        free_allies.pop(closest.id)
        
        while len(free_allies) != 0:
            closest = get_closest(open_enemies[0], free_allies)
            cover_action = cover(open_enemies[0], self.team.ball_controler)
            self.team.game.add_action(cover_action, closest.id, self.team.is_blue)
            open_enemies.pop(0)
            free_allies.remove(closest)

    def update(self):
        # state machine transition 
        if self.team.ball_controler != -1 and self.team.ball_controler.is_blue == self.team.is_blue:
            return OFFENSIVE_STRATEGY_STATE_NUMBER
            
        elif self.team.ball_controler == -1:
            return NEUTRAL_STRATEGY_STATE_NUMBER
            
        return DEFENSIVE_STRATEGY_STATE_NUMBER
