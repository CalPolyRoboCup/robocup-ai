import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/home/adleywong/Repositories/robocup-ai/src')
from basic_skills.action import *
from basic_skills.helper_functions import *
from basic_skills.move_to.move_to import *

class dribble_ball(action):
    def __init__(self):
        action.__init__(self)
        self.target_loc = False
        self.target_rot = False
        self.ball_loc = False

    def set_dribble_to(self, target_loc, target_rot):
        pass

    def PID_face_ball(self):
        pass

    def run(self):
        pass

    """Plan to move robot to location before approching dribble loc"""
    def move_to_loc(self, loc, rot):
        

    def get_ball_pos(self, game):
        self.ball_loc = game.ball.loc
