#!/usr/bin/python3

import matplotlib.pyplot as plt
from matplotlib import collections as mc
import pylab as pl
import random
import math

class rrt():
    def __init__(self, 
            start_Point, # coordinates of start Point
            goal_Point, # coordinates of goal Point
            obstacle_List, # List of obstacle ( x, y, radius )
            randomization_Constraints, # List of min/max constraints for random Point Sampling
            growth_Factor = 90, # Amount by which a new branch will grow towards Sample Point
            goal_SampleRate = 10, # probability of sampling the Goal point
            safety_margin = 25,
            ANIMATE = False) : # animation toggle

        self.start_Point = Point ( start_Point[0], start_Point[1] )
        self.goal_Point = Point ( goal_Point[0], goal_Point[1] )
        self.min_Rand_Constraint = randomization_Constraints[0]
        self.max_Rand_Constraint = randomization_Constraints[1]
        self.growth_Factor = growth_Factor
        self.goal_SampleRate = goal_SampleRate
        self.safety_margin = safety_margin
        self.obstacle_List = obstacle_List
        self.ANIMATE = ANIMATE

    def computeSolutionPath(self) :
        # 1) Initialize PointList with start_Position
        point_List = [self.start_Point]
        if self.ANIMATE : fig = plt.figure()
        # WHILE LOOP
        # self.windowCount = 0
        self.reached_Goal = False
        self.collision = False
        while ( self.reached_Goal == False ) :
            # 2) generateRandomSample (), returns random sample Point or biased Point
            sample_Point = self.generateRandomSamplePoint(self.collision)
            # 3) getClosestPointIndex( sample Point  ), returns index of Point in pointList closest to given sample point
            closest_Point_Index = self.getClosestPointIndex(point_List, sample_Point)
            # 4) growTree( new_Point , sample_Point, closest_Point_Index ) , creates new Point and grows tree at closest Point to sample in point_List
            new_Point = self.growTree(point_List, sample_Point, closest_Point_Index)
            # 4a) refresh RRT figure and draw new tree
            # 5) collisionDetected ( nearby Point ), returns boolean based on check if nearby point collides with osbtacle
            # 5a) If 5 is true , next loop iteration 
            # 5b) If 5 is false, proceed to 6
            self.collision = self.collisionDetected(new_Point, point_List, self.collision)
            if self.collision :
                self.collision = True
                continue
            self.collision = False
            # 6) add new_Point to point_List
            point_List.append(new_Point)
            if self.ANIMATE : self.drawRRT ( fig , sample_Point, point_List )
            # 7) goalStatus( new_Point) , returns boolean based on check of distance from goal_Point to new_Point
            # 7a) If 7 is true, break while loop & print rrt graph
            # 7b) if 7 is false, next loop iteration
            if self.getGoalStatus(new_Point) : self.reached_Goal = True

        # Trace backwards towards start Point for solution path
        # 8) traceFinalPath() , returns list of Point coordinate pairs from endPoint to startPoint
        if self.ANIMATE : self.traceFinalPath(point_List)
        return [ point_List[1].x, point_List[1].y]
        # self.solution_Path = traceFinalPath( point_List)
    
    ######## METHODS
    def drawRRT(self, fig, sample_Point, point_List) :
        fig.clf()
        # Animate Sampling
        plt.plot( sample_Point.x, sample_Point.y, "k*")
        # Animate RRT
        for point in point_List :
            if point.preceding_Point_Index is not None :
                plt.plot( [ point.x, point_List[point.preceding_Point_Index].x],
                        [point.y, point_List[point.preceding_Point_Index].y], "-r")
        
        fig = plt.gcf()
        ax = fig.gca() 
        for ( obstacle_x, obstacle_y, radius ) in self.obstacle_List :
            ax.add_artist(plt.Circle( (obstacle_x, obstacle_y) , radius , color='b'))
        
        plt.plot( self.start_Point.x, self.start_Point.y, "-xb", linewidth=2, markersize=12)
        plt.plot( self.goal_Point.x, self.goal_Point.y, "-xb",  linewidth=2, markersize=12)
        plt.axis( [-1, 12, 0, 12] )
        fig.show()
        plt.pause( 0.01 )

    def traceFinalPath(self ,point_List) : # work in progress
        solution_Points = [point_List[-1]]
        # solution_Coordinate_Pairs = [ [point_List[-1].x, point_List[-1].y] ]
        for this_Point in solution_Points :
            if this_Point.preceding_Point_Index != None :
                plt.plot( [ this_Point.x , point_List[this_Point.preceding_Point_Index].x ],
                        [ this_Point.y , point_List[this_Point.preceding_Point_Index].y ], "-b" )
                solution_Points.append( point_List[ this_Point.preceding_Point_Index])

        plt.show()

    def getGoalStatus(self, new_Point) :
        # Im still having problems with providing a solution that is as close as possible to goal
        dx = new_Point.x - self.goal_Point.x
        dy = new_Point.y - self.goal_Point.y
        distance = math.sqrt ( dx**2 + dy**2 )

        if distance <= self.growth_Factor :
            print("True")
            print("Start X :", self.start_Point.x)
            print("Start Y :", self.start_Point.y)

            print("X :", new_Point.x)
            print("Y :", new_Point.y)
            return True
        else :
            print("NO GOAL")
            return False

    def collisionDetected(self, new_Point, point_List, collision_Detected) :
        # first check for obstacle collision
        for (x , y, obstacle_Radius) in self.obstacle_List :
            dx = x - new_Point.x
            dy = y - new_Point.y
            distance_to_Obstacle = math.sqrt( dx**2 + dy**2)
            if distance_to_Obstacle <= obstacle_Radius :
                print("COLLISION")
                return True
        # then check for redundant point sampling
        if collision_Detected :
            for point in point_List :
                dx = point.x - new_Point.x
                dy = point.y - new_Point.y
                distance_to_Other_Point = math.sqrt( dx**2 + dy**2)
                if distance_to_Other_Point <= 0 : 
                    print("REDUNDANT")
                    return True
        return False

    def growTree(self, point_List, sample_Point, closest_Point_Index) :
        # grow branch
        growth_Angle = math.atan2((sample_Point.y - point_List[closest_Point_Index].y) , (sample_Point.x - point_List[closest_Point_Index].x))
        #print(growth_Angle)
        new_Point = Point ( point_List[closest_Point_Index].x + self.growth_Factor * math.cos( growth_Angle), 
                point_List[closest_Point_Index].y + self.growth_Factor * math.sin( growth_Angle))
        new_Point.preceding_Point_Index = closest_Point_Index

        return new_Point

    def getClosestPointIndex(self, point_List, sample_Point) :
        distance_to_Sample = []
        for this_Point in point_List :
            distance_to_Sample.append(math.sqrt((sample_Point.x - this_Point.x )**2 + (sample_Point.y - this_Point.y)**2 )) 

        index = distance_to_Sample.index( min(distance_to_Sample))
        return index

    def generateRandomSamplePoint(self, collision_Detected) : # generateRandomSample (), returns random sample Point or biased Point
        if collision_Detected : 
            #if random.randint(0,100) > self.goal_SampleRate :
            sample_Point = Point ( random.uniform( self.min_Rand_Constraint, self.max_Rand_Constraint) , random.uniform( self.min_Rand_Constraint, self.max_Rand_Constraint)) 
            #else : sample_Point = Point ( self.goal_Point.x , self.goal_Point.y) # Biased Point
        
        else : sample_Point = Point ( self.goal_Point.x , self.goal_Point.y) # Biased Point 
        #print("SAMPLE X :", sample_Point.x)
        #print("SAMPLE Y :", sample_Point.y)
        return sample_Point

class Point () :

    def __init__(self, x, y) :
        self.x = x
        self.y = y
        self.preceding_Point_Index = None
