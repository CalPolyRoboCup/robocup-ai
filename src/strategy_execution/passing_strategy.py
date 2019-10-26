import sys
import os
dirname = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, dirname)
from state_machine import state
from strategy_numbers import *

dirname = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, dirname+'/..')
from basic_skills.source.InterceptBall import InterceptBall
from basic_skills.source.GetOpen import Fielder
from basic_skills.source.Goalie import Goalie


#assigns actions to roots while the ball is being passed by this team
class passing_strategy(state):
    state_number = 3
    def __init__(self, id, team):
        state.__init__(self, id)
        self.team = team

        self.pass_timeout_time = 500
        self.pass_timeout = 0
        self.passer_id = -1

    def setup(self):
        reciever_id = self.team.pass_action.target_robot.id
        print()
        print("PASS to", "blue" if self.team.is_blue else "yellow", reciever_id)
        print()

        self.team.game.add_action(InterceptBall(),
                reciever_id, self.team.is_blue)
        self.passer_id = self.team.ball_controler.id

        if self.team.ball_controler.id == self.team.goalie.id:
            self.team.game.add_action(Goalie(), self.team.ball_controler.id, self.team.is_blue)
        else:
            fielder_action = Fielder(self.team.goalie, self.team.allies[reciever_id], 
                                    self.team.blocker_enemies, self.team.allies)
            self.team.game.add_action(fielder_action, self.team.ball_controler.id, self.team.is_blue)
    
        self.pass_timeout = 0

    def update(self):
        self.pass_timeout += 1

        if (self.team.ball_controler is not None and 
                self.team.ball_controler.id != self.passer_id):
            
            # if they have the ball
            if self.team.ball_controler.is_blue != self.team.is_blue:
                print("interception")
                return DEFENSIVE_STRATEGY_STATE_NUMBER
                
            # if we have the ball
            if (self.team.ball_controler.is_blue == self.team.is_blue):
                print("recieved", self.team.ball_controler.id)
                return OFFENSIVE_STRATEGY_STATE_NUMBER
                
        if self.pass_timeout == self.pass_timeout_time:
            print("fizzle")
            return NEUTRAL_STRATEGY_STATE_NUMBER
            
        return PASSING_STRATEGY_STATE_NUMBER