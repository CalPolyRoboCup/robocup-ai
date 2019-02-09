import numpy as np
import sys
sys.path.insert(0, '../..')
from basic_skills.action import *
from basic_skills.orbit_ball.orbit_ball import *

 #THIS IS A MINIMAL FUNCTIONING VERSION TO TEST HANDLE_BALL
class pass_to(orbit_ball):
  def __init__(self, target_robot = None, offset = 115):
    orbit_ball.__init__(self, None, offset)
    self.target_robot = target_robot
    self.epsilon = 0.1
    
  def run(self):
    # TODO predictive passing
    self.target_loc = self.target_robot.loc
    
    tvec = self.target_loc - self.robot.loc
    bvec = self.game.ball.loc - self.robot.loc
    if abs(normalize_angle(math.atan2(tvec[1], tvec[0]) - math.atan2(bvec[1], bvec[0]))) < self.epsilon and self.game.ball.controler != False:
      self.kick = 1
    else:
      self.kick = 0
    
    actions = orbit_ball.run(self)
    return actions