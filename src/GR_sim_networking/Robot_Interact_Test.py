import  Robot_Interact
import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname + '\..')
from basic_skills.source.InterceptBall import *


game = Robot_Interact.RR_sim()

game.add_action(intercept_all(0), 0, True)

while (1):
        game.step()

