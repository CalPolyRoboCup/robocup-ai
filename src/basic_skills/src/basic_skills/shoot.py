import numpy as np
import sys

from basic_skills.action import *
from basic_skills.helper_functions import *

from basic_skills.move_to import MoveTo
from pysim.PySim import *


class Shoot(MoveTo):
  GOAL_WIDTH = 1200
  FIELD_WIDTH = 12000
  FIELD_HEIGHT = 9000

  TARGET_ALIGNMENT_DISTANCE = 240
  MAXIMUM_TARGET_DISTANCE = 10
  MAXIMUM_SHOOT_DISTANCE = 130
  KICK_POWER = 100
  RAY_TEST_RESOLUTION = 10
  
  ALIGNMENT_STATE = 0
  APPROACHING_STATE = 1
  SHOOTING_STATE = 2
  DONE_STATE = 3

  def __init__(self, game):
    """
    Initializes the shoot command.
    """
    MoveTo.__init__(self, game)
    self.goal = np.array([0, 0])
    self.state = Shoot.ALIGNMENT_STATE
    self.ball_distance = None
    self.ray_targets = []
  
  def add(self, robot):
    """
    Runs initialization logic when the action is added
    to a robot.
    """
    Action.add(self, robot)
    if self._robot.is_blue:
      self.goal = np.array([Shoot.FIELD_WIDTH * 0.5, 0])
    else:
      self.goal = np.array([-Shoot.FIELD_WIDTH * 0.5, 0])

    for i in range(1, Shoot.RAY_TEST_RESOLUTION):
      self.ray_targets.append(np.array([self.goal[0],
        -Shoot.GOAL_WIDTH * 0.5 +\
        i * (Shoot.GOAL_WIDTH / Shoot.RAY_TEST_RESOLUTION)]))

  def raytest_target(self):
    """
    Finds the best place to shoot on goal to avoid the ball
    getting blocked by other robots.
    """
    max_target = None
    max_distance = -1

    for target in self.ray_targets:
      # An arbitrary high number, effectively impossible to reach naturally
      min_distance = 1000000
      direction = target - self.game.ball.loc

      for b in self.game.blue_robots:
        if self._robot.is_blue and b.id == self._robot.id:
          continue

        closest_point = drop_perpendicular(b.loc, self.game.ball.loc, direction)
        local_distance = np.linalg.norm(b.loc - closest_point)
        
        if local_distance < min_distance:
          min_distance = local_distance
          
      for b in self.game.yellow_robots:
        if not self._robot.is_blue and b.id == self._robot.id:
          continue

        closest_point = drop_perpendicular(b.loc, self.game.ball.loc, direction)
        local_distance = np.linalg.norm(b.loc - closest_point)
        
        if local_distance < min_distance:
          min_distance = local_distance

      if min_distance > max_distance:
        max_distance = min_distance
        max_target = target
    
    return max_target

  def run(self, delta_time):
    """
    Executes the run action taking into account
    the given delta time.
    """
    if self.state == Shoot.ALIGNMENT_STATE:
      target_direction = self.raytest_target() - self.game.ball.loc
      target_direction /= np.linalg.norm(target_direction)
      target = self.game.ball.loc - target_direction * Shoot.TARGET_ALIGNMENT_DISTANCE

      target_distance = np.linalg.norm(target - self._robot.loc)

      self.kick = 0

      if target_distance <= Shoot.MAXIMUM_TARGET_DISTANCE:
        self.state = Shoot.APPROACHING_STATE
    elif self.state == Shoot.APPROACHING_STATE:
        target = self.game.ball.loc
        self.kick = 0

        self.ball_distance = np.linalg.norm(self._robot.loc - self.game.ball.loc)

        if self.ball_distance < Shoot.MAXIMUM_SHOOT_DISTANCE:
          self.state = Shoot.SHOOTING_STATE
    elif self.state == Shoot.SHOOTING_STATE:
      target = self.game.ball.loc
      self.kick = Shoot.KICK_POWER

      ball_distance = np.linalg.norm(self._robot.loc - self.game.ball.loc)

      if ball_distance > self.ball_distance:
        self.state = Shoot.DONE_STATE

      self.ball_distance = ball_distance
    elif self.state == Shoot.DONE_STATE:
      return
    
    ball_direction = self.game.ball.loc - self._robot.loc

    self.set_target(target, np.arctan2(-ball_direction[1], ball_direction[0]))
    actions = MoveTo.run(self, delta_time)
    self.actions = actions

    return actions
  
  def done(self):
    """
    Returns true after the ball has been shot
    """
    return self.state == Shoot.DONE_STATE