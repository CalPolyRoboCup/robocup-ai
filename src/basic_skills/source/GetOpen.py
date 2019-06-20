import numpy as np
import math
import sys
import os
import time
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from helper_functions import mag, min_angle, drop_perpendicular, dist, angle_to, rotate_vector, scale_to, squash
from action import action
from MoveTo import MoveTo
from DribbleBall import DribbleBall
from worst_intercept import worst_intercept


class GetOpen(MoveTo):
    '''
    brief: attempts to move the robot to a location that is capable of 
    passing to each point in points points are weighted
    
    params: points  - points we wish to pass to. points[0] is assumed
                        to be the ball controler. Recommended length <= 2
            weights - relative importance of points. Must be the same
                        size as points
            enemies - list of enemy robots. Used to calculated pass values
            allies  - list of allied robots. Used to ensure robots spread out.
    '''
    def __init__(self, points, weights, enemies, allies):
        MoveTo.__init__(self)
        self.points = points
        self.weights = weights
        self.enemies = enemies
        self.allies = allies
        
        # for debugging. Keypoints to plot with pygame.step
        self.prints = [(np.array([0,0]), 1) for _ in range(len(self.weights))]
        
        # score representing how well this robot can make passes to points
        self.current_score = 0

        # used to track the enemy that blocked each shot last cycle
        # use preference_id option of worst_intercept to escape local minima
        self.last_intercept_id = [None for _ in range(len(self.weights))]

        # positioning_factors.
        self.allie_avoidance_factor = 800
        self.enemy_avoidance_factor = 0.2
        self.point_proximity_factor = 2000

    def add(self, robot, game):
        MoveTo.add(self, robot, game)
        self.lag_loc = robot.loc
        
    '''
    brief: checks passes from location to self.points and generates a rating

    params: location to rate
    returns: 
        improvements - a point that should be better than location
        score - a score for location. Bigger is betterer
    '''
    def AnalyzePoint(self, location):
        '''
        long:
            R - robot
            P - point
            E - enemy with best chance of intercepting pass to P
            Y - lable for the local Y dimension
            X - lable for the local X dimension
            r - rougly where improvement should be

                    E  Y        P
                     `.    ..--``
                     ..--``
                R--``X
                   r

            rotate away from E and move towards P
            
            We will then make a weighted average of the result of 
            this procedure on every point in self.points and return
            it as improvements

            After that is all done make sure you aren't moving close
            to any of your allies
        '''
        improvements = location
        improv_weight = 1
        pind = 0
        score = 0
        for p in self.points:
            worst, self.last_intercept_id[pind] = worst_intercept(location, p, self.enemies, self.last_intercept_id[pind])
            importance = 0.1
            improvement = np.array([0,0])
            '''
            Something is happening
            rotate away from worst enemy
            rotate farthur for worse interceptions
            '''
            if worst is not None:
                importance += (1 - squash(abs(worst)))
                point_vector = location - p
                straff_angle = self.enemy_avoidance_factor * importance * self.weights[pind]
                if worst <= 0:
                    improvement = rotate_vector(point_vector, straff_angle)
                else:
                    improvement = rotate_vector(point_vector, -straff_angle)
                improvement = improvement + p

            '''
            prefer to be close to the points
            '''
            tuck_factor = self.point_proximity_factor * (1 - importance) * self.weights[pind]
            improvement = improvement + scale_to(p - location, tuck_factor)

            improvements = improvements + improvement * importance
            improv_weight += importance

            # debug
            self.prints[pind] = (improvement, 10*importance)
            
            if worst is None:
                score += 1
            else:
                score += squash(abs(worst), 600)
            pind += 1
        
        '''
        weighted average of improvements
        '''
        improvements = improvements/improv_weight
                
        '''
        don't stand too close to your allies
        '''
        for a in self.allies:
            allie_pressure = (a.loc - location)
            pressure_norm = mag(allie_pressure)
            allie_push = (self.allie_avoidance_factor * (1 - squash(pressure_norm)))
            allie_pressure = scale_to(allie_pressure, allie_push)
            improvements = improvements - allie_pressure

        return improvements, score
        
    def get_targets(self):
        improvement, score = self.AnalyzePoint(self.robot.loc)
        self.current_score = score

        target_loc = improvement
        target_rot = angle_to(self.points[0], self.robot.loc)
        return target_loc, target_rot

    def run(self):
        target_loc, target_rot = self.get_targets()
        self.set_target(target_loc, target_rot)
        return MoveTo.run(self)


class GetOpenHolistic(GetOpen):
    '''
    GetOpen logic, but if the current position gets too bad
    the robot will make a high level decision about where
    it should be and go there
    '''
    def __init__(self, points, weights, enemies, allies):
        GetOpen.__init__(self, points, weights, enemies, allies)

        '''
        if score drops bellow threshold for count frames 
        GetOpen_at a better location
        '''
        self.freakout = 0
        self.freakout_count = 20
        self.freakout_threashold = 1.3
        self.repositioning = False

        self.repositiong_cycles = 10
        self.repositioning_factor = 5
        self.repositioning_radius = 200
    
    def GetOpen_at(self, location):
        '''
        start at location.
        Move a bit towards where you are
        Take repositioning cycles steps using the GetOpen 
            procedure and boost the step size by repositioning factor
        '''
        location = location + scale_to(self.robot.loc - location, self.repositioning_radius)
        for _ in range(self.repositiong_cycles):
            new_location, _ = self.AnalyzePoint(location)
            location = location + self.repositioning_factor * (new_location - location)
        return location

    def run(self):
        if not self.repositioning:
            if self.current_score < self.freakout_threashold:
                self.freakout += 1
            if self.freakout >= self.freakout_count:
                good_location = self.GetOpen_at(self.points[0])
                self.set_target(good_location, self.robot.rot)
                print("freakout", self.robot.id, self.target_loc, good_location)
                self.repositioning = True

        if self.repositioning:
            if dist(self.robot.loc, self.target_loc) < self.repositioning_radius:
                self.repositioning = False
                self.freakout = 0
                print("freakout done", self.robot.id)
            return MoveTo.run(self)
        return GetOpen.run(self)

class Striker(GetOpenHolistic):
    '''
    forward support class. Gets open for ball controler and shot at the goal
    '''
    def __init__(self, ball_controller, goal, enemies, allies):
        GetOpenHolistic.__init__(self, [], [0.6, 0.4], enemies, allies)
        self.goal = goal
        self.ball_controller = ball_controller
        self.allies = allies
        self.enemies = enemies

    def update_points(self):
        self.points = [self.ball_controller.loc, self.goal]

    def add(self, robot, game):
        self.update_points()
        GetOpen.add(self, robot, game)

    def run(self):
        self.points[0] = self.ball_controller.loc
        return GetOpenHolistic.run(self)

class Fielder(GetOpenHolistic):        
    '''
    Rear support class. Gets open for ball_controller 
    and anouther allied robot (currently one of the Strikers)
    '''
    def __init__(self, ball_controller, target_to_support, enemies, allies):
        GetOpenHolistic.__init__(self, [], [0.6, 0.4], enemies, allies)
        self.target_to_support = target_to_support
        self.ball_controller = ball_controller
        self.allies = allies
        self.enemies = enemies

    def update_points(self):
        self.points = [self.ball_controller.loc, self.target_to_support.loc]

    def add(self, robot, game):
        self.update_points()
        GetOpen.add(self, robot, game)

    def run(self):
        self.points[0] = self.ball_controller.loc
        self.points[1] = self.target_to_support.loc
        return GetOpenHolistic.run(self)
        
class GetOpenForKick(GetOpen):
    '''
    rewraps GetOpen, but with DribbleBall for control
    GetOpenHolistic is not used because abandoning the
    ball is not good
    '''
    def __init__(self, support_robots, enemies, allies):
        GetOpen.__init__(self, [a.loc for a in support_robots], [1 for a in support_robots], enemies, allies)
        DribbleBall.__init__(self)
        self.allies = allies
        self.enemies = enemies
        self.support_robots = support_robots
        
        self.freakout_weight = -10
        self.lag_factor = 0.5
        self.depth_weight = 0.01

    def update_points(self):
        for i in range(len(self.support_robots)):
            self.points[i] = self.support_robots[i].loc

    def add(self, robot, game):
        self.update_points()
        GetOpen.add(self, robot, game)

    def run(self):
        self.update_points()
        target_loc, _ = self.get_targets()
        DribbleBall.set_target(self, target_loc)
        return DribbleBall.run(self)