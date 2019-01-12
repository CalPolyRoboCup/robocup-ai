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
            ball_Point, # coordinates of ball Point
            obstacle_List, # List of obstacle ( x, y, radius )
            X_randomization_Constraints, # List of min/max constraints for random Point Sampling
            Y_randomization_Constraints, # List of min/max constraints for random Point Sampling 
            growth_Factor = 180, # Amount by which a new branch will grow
            goal_SampleRate = 5, # probability of sampling the Goal point
            MAX_ITER = 50, # maximum num of script iterations
            ANIMATE = False) : # animation toggle

        self.start_Point = Point ( start_Point[0], start_Point[1] )
        self.goal_Point = Point ( goal_Point[0], goal_Point[1] )
        self.ball_Point = Point ( ball_Point[0], ball_Point[1] )
        self.X_Rand_Constraints = X_randomization_Constraints
        self.Y_Rand_Constraints = Y_randomization_Constraints
        # Hard coded attributes
        self.growth_Factor = growth_Factor
        self.goal_SampleRate = goal_SampleRate
        self.obstacle_List = obstacle_List
        self.MAX_ITER = MAX_ITER
        self.ANIMATE = ANIMATE

    def computeSolutionPath(self) :
        #print("********** NEW RRT")
        # point_List contains RRT points
        point_List = [self.start_Point]
        if self.ANIMATE : fig = plt.figure()
       
        iterations = 0
        skip_rrt = True
        reached_Goal = False
        collision = False
        while iterations < self.MAX_ITER :
            iterations+=1 
            # Point = generateRandomSample ( bool )
            # returns random sample Point or biased Point
            sample_Point = self.generateRandomSamplePoint( skip_rrt )
            # int = getClosestPointIndex( Point list , Point )
            # returns index of Point in pointList closest to sample point
            closest_Point_Index = self.getClosestPointIndex(point_List, sample_Point)
            # Point = growTree( Point , Point, int )
            # Grows branch from closest Point to sample Point in point_List, and returns new Point
            new_Point = self.growTree(point_List, sample_Point, closest_Point_Index)
            # bool = collisionDetected ( Point, Point list, bool )
            # returns boolean based on check if new point collides with osbtacle 
            # or if new Point is redundant
            collision = self.collisionDetected(new_Point, point_List, skip_rrt)
            if collision :
                skip_rrt = False
                continue
            if skip_rrt : break

            point_List.append(new_Point)
            if self.ANIMATE : self.drawRRT ( fig , sample_Point, point_List )
            # bool = goalStatus( Point )
            # returns boolean based on check of distance from goal_Point to new_Point
            if self.getGoalStatus(new_Point) : reached_Goal = True
            if reached_Goal : break
        
        # Animates RRT generation & solution path using Matplotlib

        if skip_rrt or (len(point_List) == 1) :
            #print("SKIP")
            return [ self.goal_Point.x , self.goal_Point.y ]
        else :
            print("TRACE")
            next_Point = self.traceFinalPath( point_List )
            return [ next_Point.x, next_Point.y ]
    
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
        closest_Point_Index = self.getClosestPointIndex(point_List, self.goal_Point)
        solution_Points = [point_List[closest_Point_Index]]
        for this_Point in solution_Points :
            if this_Point.preceding_Point_Index != None :
                if self.ANIMATE : 
                    plt.plot( 
                            [ this_Point.x , point_List[this_Point.preceding_Point_Index].x ],
                            [ this_Point.y , point_List[this_Point.preceding_Point_Index].y ], 
                            "-b" 
                            )
                solution_Points.append( point_List[ this_Point.preceding_Point_Index])

        if self.ANIMATE : plt.show()
        return solution_Points[-1]

    def getGoalStatus(self, new_Point) :
        # Im still having problems with providing a solution that is as close as possible to goal
        dx = new_Point.x - self.goal_Point.x
        dy = new_Point.y - self.goal_Point.y
        distance = math.sqrt ( dx**2 + dy**2 )

        #if distance <= (self.growth_Factor):
        if distance <= 90 :
            #print("Start X :", self.start_Point.x)
            #print("Start Y :", self.start_Point.y)

            #print("X :", new_Point.x)
            #print("Y :", new_Point.y)
            return True
        else :
            #print("NO GOAL")
            return False

    def collisionDetected(self, new_Point, point_List, skip_rrt) :
        # first check for obstacle collision
        for (x , y, obstacle_Radius) in self.obstacle_List :
            dx = x - new_Point.x
            dy = y - new_Point.y
            distance_to_Obstacle = math.sqrt( dx**2 + dy**2)
            if distance_to_Obstacle <= 2*obstacle_Radius : # accounts for self radius & other robot
                print("COLLISION")
                return True
        # WORK IN PROGRESS
        # then check for redundant point sampling
        #if skip_rrt == False :
        #    for point in point_List :
        #        dx = point.x - new_Point.x
        #        dy = point.y - new_Point.y
        #        distance_to_Other_Point = math.sqrt( dx**2 + dy**2)
        #        if distance_to_Other_Point <= self.growth_Factor : 
        #            print("REDUNDANT")
        #            return True
        return False

    def growTree(self, point_List, sample_Point, closest_Point_Index) :
        # grow branch
        growth_Angle = math.atan2(
                (sample_Point.y - point_List[closest_Point_Index].y) , 
                (sample_Point.x - point_List[closest_Point_Index].x)
                )
        #print(growth_Angle)
        new_Point = Point ( 
                point_List[closest_Point_Index].x + self.growth_Factor * math.cos( growth_Angle), 
                point_List[closest_Point_Index].y + self.growth_Factor * math.sin( growth_Angle)
                )
        new_Point.preceding_Point_Index = closest_Point_Index
        return new_Point

    def getClosestPointIndex(self, point_List, sample_Point) :
        distance_to_Sample = []
        for this_Point in point_List :
            distance_to_Sample.append( 
                    math.sqrt( (sample_Point.x - this_Point.x )**2 + (sample_Point.y - this_Point.y)**2 )
                    ) 
        index = distance_to_Sample.index( min(distance_to_Sample))
        return index

    def generateRandomSamplePoint(self, skip_rrt) : # generateRandomSample (), returns random sample Point or biased Point
        if skip_rrt == False : 
            if random.randint(0,100) > self.goal_SampleRate :
                sample_Point = Point ( 
                        random.uniform( self.X_Rand_Constraints[0], self.X_Rand_Constraints[1]) , 
                        random.uniform( self.Y_Rand_Constraints[0], self.Y_Rand_Constraints[1])
                        )
                return sample_Point
        
        sample_Point = Point ( self.goal_Point.x , self.goal_Point.y) # Biased Point 
        # print("SAMPLE X :", sample_Point.x)
        # print("SAMPLE Y :", sample_Point.y)
        return sample_Point

class Point () :

    def __init__(self, x, y) :
        self.x = x
        self.y = y
        self.preceding_Point_Index = None
