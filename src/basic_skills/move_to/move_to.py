import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/home/wulfkine/repos/robocup-ai/src')
from basic_skills.action import *
from basic_skills.helper_functions import *
from basic_skills.rrt.rrt import *

import numpy as np

class move_to(action):
  def __init__(self, target_loc = False, target_rot = False):
    action.__init__(self)
    self.target_loc = target_loc # boolean, why? placeholder?
    self.target_rot = target_rot # boolean

    self.translational_control_speed = 150
    self.locP = 34.3#.0045
    self.locD = 15#.00075
    self.pivot_center = np.array([-.1,0])
    self.pivot_factor = 0#.0004

    self.rot_control_speed = 29.81
    self.rotP = 8
    self.rotD = 5.89

  def set_target(self, target_loc, target_rot):
    #self.target_loc = target_loc
    #self.target_rot = target_rot
    
    # RRT test
    my_Robots = []
    #for robot in self.game.yellow_robots :
    #    if (self.robot.loc[0] != robot.loc[0]) & (self.robot.loc[1] != robot.loc[1]) :
    #        my_Robots.append( (robot.loc[0], robot.loc[1], 25 ) )
    for robot in self.game.blue_robots :
        my_Robots.append( (robot.loc[0], robot.loc[1], 90 ) )

    rrt_strategy = rrt(
            [self.robot.loc[0], self.robot.loc[1]],
            target_loc,
            my_Robots,
            [0,10])
    next_Point = rrt_strategy.computeSolutionPath()
    self.next_Point = np.array( next_Point, dtype = np.float64)
    self.target_loc = self.next_Point

  def run(self):
    norm_vel, tang_vel = self.PID_loc() #
    rot_vel = self.PID_rot()
    action = [0,0,norm_vel, tang_vel, rot_vel]
    self.action = action
    #print("move to P - ", self.locP)
    return action

  def PID_loc(self):
    vector = self.target_loc - self.robot.loc
    distance = np.linalg.norm(vector)
    local = convert_local(vector, -self.robot.rot)
    
    local_vel = convert_local(self.robot.velocity, -self.robot.rot)
    loc_PID = self.locP*local - self.locD*local_vel
    #print(self.locP*local, -local_vel*self.locD)
    pidMag = np.linalg.norm(loc_PID)
    if pidMag > self.translational_control_speed:
      loc_PID *= self.translational_control_speed/pidMag

    #correct for induced twist
    #pivot_vel = local_vel - self.pivot_center
    #cross_prod = pivot_vel[0] * loc_PID[1] - pivot_vel[1] * loc_PID[0]
    #rot_vel = cross_prod * self.pivot_factor
    #print(rot_vel, self.locP*local, -self.locD*local_vel)
    return loc_PID[0], loc_PID[1]#, rot_vel

  def PID_rot(self):
    distance = min_angle(self.target_rot - self.robot.rot)
    PID = self.rotP*distance - self.rotD*self.robot.rot_vel
    if (PID > self.rot_control_speed):
      PID = self.rot_control_speed
    elif(PID < -self.rot_control_speed):
      PID = -self.rot_control_speed

    return PID

  def done(self):
    epsilon = 5
    if np.linalg.norm(self.target_loc - self.robot.loc) < epsilon and np.linalg.norm(self.robot.velocity) < epsilon:
      return True
    return False
