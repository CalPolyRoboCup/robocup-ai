import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/home/nathan/Documents/robocup2018/robocup-ai/src')
from basic_skills.action import *
from basic_skills.helper_functions import *

import numpy as np

class move_to(action):
  def __init__(self):
    action.__init__(self)
    self.target_loc = False
    self.target_rot = False

    self.translational_control_speed = 13
    self.locP = .0045
    self.locD = .00075
    self.pivot_center = np.array([-.1,0])
    self.pivot_factor = .0004

    self.rot_control_speed = 65
    self.rotP = 8
    self.rotD = .22
  def set_target(self, target_loc, target_rot):
    self.target_loc = target_loc
    self.target_rot = target_rot
  def run(self):
    norm_vel, tang_vel, rot_vel = self.PID_loc()
    rot_vel = self.PID_rot(rot_vel)
    action = [0,0,norm_vel, tang_vel, rot_vel]
    self.action = action
    return action
  def convert_local(self, direction, rot):
    norm = direction[0]*math.cos(-rot) - direction[1]*math.sin(-rot)
    tang = direction[1]*math.cos(-rot) + direction[0]*math.sin(-rot)
    return np.array([norm, tang])
  def PID_loc(self):
    vector = self.target_loc - self.robot.loc
    distance = np.linalg.norm(vector)
    local = self.convert_local(vector, self.robot.rot)
    
    local_vel = self.convert_local(self.robot.velocity, self.robot.rot)
    loc_PID = self.locP*local - self.locD*local_vel
    pidMag = np.linalg.norm(loc_PID)
    if pidMag > self.translational_control_speed:
      loc_PID *= self.translational_control_speed/pidMag

    #correct for induced twist
    pivot_vel = local_vel - self.pivot_center
    cross_prod = pivot_vel[0] * loc_PID[1] - pivot_vel[1] * loc_PID[0]
    rot_vel = cross_prod * self.pivot_factor
    #print(rot_vel, self.locP*local, -self.locD*local_vel)
    return loc_PID[0], loc_PID[1], rot_vel
  def PID_rot(self, rot_vel):
    distance = min_angle(self.target_rot - self.robot.rot)
    PID = self.rotP*distance - self.rotD*self.robot.rot_vel
    PID += rot_vel
    if (PID > self.rot_control_speed):
      PID = self.rot_control_speed
    elif(PID < -self.rot_control_speed):
      PID = -self.rot_control_speed

    return PID
