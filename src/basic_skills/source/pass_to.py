import numpy as np
import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from action import *
from orbit_ball import *

class pass_to(orbit_ball):
  def __init__(self, target_robot = None, offset = 110):
    orbit_ball.__init__(self, None, offset)
    self.target_robot = target_robot
    
    # maximum angle in radians between current kick vector and target kick vector that
    # is acceptable for a pass
    self.epsilon = 0.15
    
    #we wait a few cycles once the ball is in position to ensure the ball alignment with the kicker is good
    self.deliberation = 0
    self.deliberation_time = 20
    
    #how much we lead the target we are kicking to
    self.lead_factor = 1/1500
    
  def run(self):
    #lead the target we are passing to
    self.target_loc = self.target_robot.loc + self.target_robot.velocity * self.lead_factor * np.linalg.norm(self.target_robot.loc - self.robot.loc)
    
    tvec = self.target_loc - self.robot.loc
    bvec = self.game.ball.loc - self.robot.loc
    
    #if we have the ball in position
    if abs(normalize_angle(math.atan2(tvec[1], tvec[0]) - math.atan2(bvec[1], bvec[0]))) < self.epsilon and self.game.ball.controler != False:
      #wait a bit then kick
      self.deliberation -= 1
      if self.deliberation <= 0:
        self.kick = 1
        self.deliberation = self.deliberation_time
    else:
      #otherwise don't kick
      self.kick = 0
      self.deliberation = self.deliberation_time
      
    #turn and face the target loc
    #note this is dependent on self.target_loc being set
    actions = orbit_ball.run(self)
    return actions
    
  def done(self):
    #this will return true when the ball has been kicked
    if self.kick:
      self.kick = 0
      return True
    return False