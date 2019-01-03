#!/usr/bin/python3
from rrt import *

constraints = [ 0.0, 11.0 ]

obstacleConstraints = [ 2.0, 10.0 ]

obstacleWall = [ ( 5.0 , 9.0 , 2.0 ),
        (5.0 , 6.0 , 1.5),
        (5.0 , 4.0 , 1.5),
        (7.0 , 4.0 , 1.5),
        (9.0 , 3.0 , 1.0)]

randomObstacles =  [ ( random.uniform( obstacleConstraints[0], obstacleConstraints[1]), random.uniform( obstacleConstraints[0], obstacleConstraints[1] ), 1.0 ) , 
        ( random.uniform( obstacleConstraints[0], obstacleConstraints[1]), random.uniform( obstacleConstraints[0], obstacleConstraints[1] ), 1.5 ) , 
        ( random.uniform( obstacleConstraints[0], obstacleConstraints[1]), random.uniform( obstacleConstraints[0], obstacleConstraints[1]), 1 ) ]

myRRT = rrt( 
        [ 1.0 , 1.0 ] ,
        [ 7.0 , 7.0 ] ,
        obstacleWall ,
        constraints )

nextPoint = myRRT.computeSolutionPath()
print("Next X : ", nextPoint[0])
print("Next Y : ", nextPoint[1])
