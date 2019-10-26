import sys
import os
dirname = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, dirname)
from action import action
from helper_functions import (mag, put_in_bounds,
            drop_perpendicular, min_angle, convert_local,
            normalize_angle, squash, dist, mag_with, scale_to)

import numpy as np


'''
Simple Proportional Differential that controls a robot to a target location 
There is no I term
'''
class MoveTo(action):
    def __init__(self, target_loc = np.array([0,0]), target_rot = False):
        action.__init__(self)
        self.target_loc = target_loc
        self.target_rot = target_rot

        # translational control parameters
        self.translational_control_speed = 150
        self.locP = 60
        self.locD = 22

        # rotational control parameters
        self.rot_control_speed = 100
        self.rotP = 200
        self.rotD = 50
        
        self.short_planning_distance = 1000
        self.safety_distance = 170

        self.plan_path = True

    def add(self, robot, game):
        #turn off path planning for goalie. TODO make this more legit
        self.plan_path = True if robot.id != 5 else False
        action.add(self, robot, game)
        
    def set_target(self, target_loc, target_rot):
        '''
        set target location and rotation. If location is invalid
        and path planning is on the location will be set in bounds
        first
        '''
        if self.plan_path:
            self.target_loc, _ = put_in_bounds(target_loc)
        else:
            self.target_loc = target_loc
        self.target_rot = target_rot
        
    def collision_avoidance(self, planned_target):
        lvec = planned_target - self.robot.loc
        lmag2 = mag(lvec) + self.safety_distance * 2
        
        for b in self.game.blue_robots:
            if self.robot.is_blue and b.id == self.robot.id:
                continue
            closest_point = drop_perpendicular(b.loc, self.robot.loc, lvec)
            local_distance = mag_with(closest_point - self.robot.loc, lvec)
            if mag(closest_point - b.loc) < self.safety_distance and local_distance > 0 and local_distance < lmag2:
                avoid_close = scale_to(self.robot.loc - b.loc, self.safety_distance)
                avoid_far = scale_to(closest_point - b.loc, self.safety_distance)
                return closest_point + avoid_close + avoid_far
                
        for b in self.game.yellow_robots:
            if not self.robot.is_blue and b.id == self.robot.id:
                continue
            closest_point = drop_perpendicular(b.loc, self.robot.loc, lvec)
            local_distance = mag_with(closest_point - self.robot.loc, lvec)
            if mag(closest_point - b.loc) < self.safety_distance and local_distance > 0 and local_distance < lmag2:
                avoid_close = scale_to(self.robot.loc - b.loc, self.safety_distance)
                avoid_far = scale_to(closest_point - b.loc, self.safety_distance)
                return closest_point + avoid_close + avoid_far

        return planned_target
        
    def plan_for_target(self):
        planned_target = self.collision_avoidance(self.target_loc)
        target_vec = planned_target - self.robot.loc
        if mag(target_vec) < self.short_planning_distance * 2:
            planned_target, _ = put_in_bounds(planned_target)
        else:
            planned_target = self.robot.loc + target_vec * self.short_planning_distance / mag(target_vec)
            planned_target, fixed = put_in_bounds(planned_target)
        return planned_target

    def run(self):
        if self.plan_path:
            planned_target = self.plan_for_target()
        else:
            planned_target = self.target_loc

        self.norm_vel, self.tang_vel = self.PID_loc(planned_target)
        self.rot_vel = self.PID_rot()
        return action.run(self)
        
    def PID_loc(self, planned_target):
        vector = planned_target - self.robot.loc
        local = convert_local(vector, -self.robot.rot)
        
        local_vel = convert_local(self.robot.velocity, -self.robot.rot)
        loc_PID = self.locP*local - self.locD*local_vel
        pidMag = mag(loc_PID)
        if pidMag > self.translational_control_speed:
            loc_PID *= self.translational_control_speed/pidMag

        return loc_PID[0], loc_PID[1]

    def PID_rot(self):
        distance = min_angle(self.target_rot - self.robot.rot)
        PID = self.rotP*distance - self.rotD*self.robot.rot_vel
        if (PID > self.rot_control_speed):
            PID = self.rot_control_speed
        elif(PID < -self.rot_control_speed):
            PID = -self.rot_control_speed

        return PID

    def done(self):
        epsilon = 15
        if (mag(self.target_loc - self.robot.loc) < epsilon and mag(self.robot.velocity) < 
              epsilon and abs(normalize_angle(self.robot.rot - self.target_rot)) < 0.1):
            return True
        return False
