import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname + '/..')
from basic_skills.helper_functions import min_angle
import numpy as np
import time
import math

from basic_skills.action import CommandStatus

import logging

class Robot:

  """
  Decodes and renders robot packets from grSim, and manages actions to send back to grSim for a specific robot
  """
  RADIUS = 90
  ORIENTATION_P = 10.0
  ORIENTATION_I = 0.0
  ORIENTATION_D = 0.25
  MAX_ANGULAR_OUT = math.pi * 4
  OUTLINE_SIZE = 1
  KICKER_THICKNESS = 30
  KICKER_WIDTH = 0.6
  KICKER_OFFSET = 10
  KICKER_COLOR = (127, 127, 127)
  YELLOW_TEAM_COLOR = (255, 255, 0)
  BLUE_TEAM_COLOR = (0, 0, 255)
  def __init__(self, is_blue, idNum, game, is_virtual_robot = False):
    """Initializes a robot object is_blue - boolean idNum - int game - grsim"""
    
    self.logger = logging.getLogger(__name__)
    
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
    self.smoothing = 0
    self.first = True
    self.last_timestamp = 0
    self.kick_cooldown = 0
    self.task = 0
    self.is_virtual_robot = is_virtual_robot
    
    self._default_action = None
    self._waiting_action = None
    self._action = None

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

  def get_action (self):
    return self._action
    
  def run_action(self, delta_time):
    """
    Checks to see if there are any state changes to the active action, then updates the current action
    :param delta_time: The time passed since the last update
    """
    #print(88, self._waiting_action, self._action)
    action_result = [0]*5
    if self._waiting_action is not None:
      # If a new action is waiting to be executed, interrupt any current actions and start execution of
      # the new one
      if self._action is not None:
          self._action.end(CommandStatus.INTERRUPTED)

      self._action = self._waiting_action
      self._waiting_action = None
      self._action.start()

    if self._action is not None:
      # Update the current action if valid
      status = self._action.get_status()

      if status == CommandStatus.RUNNING:
        action_result = self._action.run(delta_time)
      else:
        self._action.end(status)
        self._action = None

    if self._action is None and self._waiting_action is None and self._default_action is not None:
      # If there is not action running and no actions are waiting, run the default action
      self._action = self._default_action
      self._action.start()
      
    return action_result
    
  def set_default_action(self, action):
    """
    Sets the robot's default action - the action will be executed if all other actions have terminated
    :param action: The action to be defined as the default
    """
    if action.get_robot() == self or action.set_robot(self):
      self._default_action = action
    else:
      self.logger.log(logging.WARNING, "Cannot set a default Command that's already been assigned"
                                             "to another Robot!")

  def add_action(self, action):
    """
    Runs the given action on the robot
    :param action: The action to run on the robot
    """
    if action.get_robot() == self or action.set_robot(self) or self.is_virtual_robot:
      if self._action is not None:
        self._waiting_action = action
      else:
        self._action = action
      action.add(self)
    else:
      self.logger.log(logging.WARNING, "Cannot run a Command that's already been assigned to"
                                           "another robot!")
