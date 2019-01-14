import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/home/dpyryev/Desktop/robocup_ai/robocup-ai/src')
from basic_skills.action import *
from basic_skills.helper_functions import *

from math import *
import numpy as np

class kick(action):
  def __init__(self):
    action.__init__(self)
    self.target_loc = False
    self.target_rot = False

#    self.translational_control_speed = 13
#    self.locP = .0045
#    self.locD = .00075
#    self.pivot_center = np.array([-.1,0])
#    self.pivot_factor = .0004
#
#    self.rot_control_speed = 65
#    self.rotP = 8
#    self.rotD = .22
  
  def set_target(self, target_loc, game):
    self.target_loc = target_loc
    self.game = game
  
  def run(self):
    action = [6.5, 0, 0, 0, 0, 0]
    self.action = action
    return action

# Below are helper functions to position the robot at the
# proper location and angle.
def calc_angle(kick_to_loc, ball_loc):
    desx = kick_to_loc[0]
    desy = kick_to_loc[1]
    ballx = ball_loc[0]
    bally = ball_loc[1]
    shift = 0

    xdiff = desx - ballx
    ydiff = desy - bally

    if (xdiff == 0) & (ydiff < 0):
        return (-1 * pi/2.0)
    elif (xdiff == 0) & (ydiff > 0):
        return pi/2.0

    if (xdiff < 0) & (ydiff == 0):
        return (-1 * pi)
    elif (xdiff > 0) & (ydiff == 0):
        return 0
    
    if (xdiff < 0) & (ydiff > 0):
        shift = pi
    elif (xdiff < 0) & (ydiff < 0):
        shift = -1 * pi

    if (xdiff == 0) & (ydiff == 0):
        return 0

    return shift + atan(ydiff/xdiff)

def calc_kick_setup(kick_to_loc, ball_loc):
    epsilon = 103 # distance difference between center of robot and ball
    angle = calc_angle(kick_to_loc, ball_loc)
    reflect = 1

    if kick_to_loc[0] < ball_loc[0]:
        reflect = -1

    robotx = ball_loc[0] - (epsilon * cos(angle))
    roboty = ball_loc[1] - (epsilon * sin(angle))
    return [robotx, roboty]

def rob_ball_dis(ball_loc, robot_loc):
    diffx = ball_loc[0] - robot_loc[0]
    diffy = ball_loc[1] - robot_loc[1]
    return sqrt((diffx ** 2) + (diffy ** 2))
