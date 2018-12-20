import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/Users/nathan/Documents/robocup-ai/src')
from basic_skills.helper_functions import min_angle
import numpy as np
import time

class robot:
  def __init__(self, is_blue, idNum, game):
    """Initializes a robot object is_blue - boolean idNum - int game - grsim"""
    self.game = game
    self.loc = np.array([0,0])
    self.rot = 0
    self.rot_vel = 0
    self.observed = 0
    self.velocity = np.array([0,0])
    self.is_blue = is_blue
    self.id = idNum
    self.last_time = time.time()
    #I don't want to make a RNN because it slows down training
    #I want to use a low pass running average filter to get 
    #velocity and feed it to a linear model maybe convolutional 
    #if we want to share weights for low level robot operations
    self.smoothing = 0.5
    self.first = True
    self.last_timestamp = 0
    self.action = None
    self.kick_cooldown = 0

  def update(self, nloc, nrot, obs, time_elapsed = 1.0/60, time_stamp = None):
    """updates robot variables nloc - np.array nrot - int obs - ? time_elaspsed - default 1/60 second time_stamp - default None"""
    #print("up")
    #print(time_stamp, self.last_timestamp, (time_stamp == None) or (time_stamp != self.last_timestamp))
    
    self.observed = obs
    if ((time_stamp == None) or (time_stamp != self.last_timestamp)) and obs and np.abs(nloc[0]) < 6000 and np.abs(nloc[1]) < 4000:
      #print("update", obs, self.id, self.is_blue, self.loc, nloc)      
      self.last_timestamp = time_stamp
      self.velocity = (self.velocity * self.smoothing + 
        (1-self.smoothing) * (nloc - self.loc)/time_elapsed)
      delta_rot = min_angle(nrot - self.rot)
      self.rot_vel = (self.rot_vel * self.smoothing + 
        (1-self.smoothing) * (delta_rot)/time_elapsed)
      self.loc = ((self.loc + self.velocity*time_elapsed) * self.smoothing + 
        (1-self.smoothing) * (nloc))
      self.rot = ((self.rot + self.rot_vel*time_elapsed) * self.smoothing + 
        (1-self.smoothing) * (nrot))

  def add_action(self, action):
    """Add an action to be ran in the wrapper action - action object"""
    self.action = action
    action.add(self, self.game)

  def run_action(self):
    """returns action when called"""
    if self.action == None or self.action.done():
      return None
    return self.action.run()
