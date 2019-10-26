import sys
import os
dirname = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, dirname)
from helper_functions import min_angle, rotate_vector
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
    self.smoothing = 0
    self.first = True
    self.last_timestamp = 0
    self.action = None
    self.kick_cooldown = 0
    self.facing = np.array([1,0])

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
    self.facing = rotate_vector(np.array([1,0]), self.rot)

  def add_action(self, action):
    """Add an action to be ran in the wrapper action - action object"""
    self.action = action
    action.add(self, self.game)

  def run_action(self):
    """returns action when called"""
    if self.action == None or (self.action.done() and self.action.terminate_when_done):
      return None
    return self.action.run()
