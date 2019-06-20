import numpy as np
import math
import time
import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/..')
from basic_skills.source.robot import robot
from basic_skills.source.InterceptBall import InterceptBall
from basic_skills.source.GetOpen import Striker, Fielder
from basic_skills.source.PassTo import PassTo
from basic_skills.source.helper_functions import travel_time, dist

class team:
    def __init__(self, game, is_blue):
        self.is_blue = is_blue
        self.game = game
        self.blocker_enemies = [robot(not is_blue, -1, None) for _ in range(4)]
        if is_blue:
            self.allies = game.blue_robots
            self.enemies = game.yellow_robots
            self.my_goal = game.blue_goal_loc
            self.enemy_goal = game.yellow_goal_loc
            self.blocker_enemies[2].loc = self.enemy_goal + np.array([240, -self.game.goal_height/2 - 500])
            self.blocker_enemies[3].loc = self.enemy_goal + np.array([240, -self.game.goal_height/2 + 500])
        else:
            self.enemies = game.blue_robots
            self.allies = game.yellow_robots
            self.my_goal = game.yellow_goal_loc
            self.enemy_goal = game.blue_goal_loc
            self.blocker_enemies[2].loc = self.enemy_goal + np.array([-240, -self.game.goal_height/2 - 500])
            self.blocker_enemies[3].loc = self.enemy_goal + np.array([-240, -self.game.goal_height/2 + 500])
        self.blocker_enemies[0].loc = self.enemy_goal + np.array([0, self.game.goal_height/2 + 500])
        self.blocker_enemies[1].loc = self.enemy_goal + np.array([0, -self.game.goal_height/2 - 500])
        self.blocker_enemies.extend(self.enemies)
        self.goalie = self.allies[-1]
        self.field_players = self.allies[:-1]

        self.ball_controler = -1
        self.ball_control_radius = 250
        self.ball_control_frames = 10
        self.ball_control_votes = 0

        self.pass_action = PassTo()
        self.intercept_action = InterceptBall()
        
        self.prints = []
    def update(self):
        # update ball_controler
        ball_controler = -1
        best = self.ball_control_radius
        for a in self.allies:
            distance = dist(a.loc, self.game.ball.loc)
            if distance < best:
                ball_controler = a
                best = distance

        for e in self.enemies:
            distance = dist(e.loc, self.game.ball.loc)
            if distance < best:
                ball_controler = e
                best = distance

        if ball_controler != self.ball_controler:
            self.ball_controler_votes += 1
            if self.ball_control_votes >= self.ball_control_votes:
                self.ball_controler = ball_controler
        else:
            self.ball_controler_votes = 0

def get_closest(enemy, free_allies):
    best_dist = travel_time(enemy, free_allies[0].loc)
    best = free_allies[0]
    for fa in free_allies[1:]:
        dist = travel_time(enemy, fa.loc)
        if dist < best_dist:
            best = fa
            best_dist = dist
    return best

def assign_Strikers_and_Fielders(team, free_allies):
    '''
    assign 2 strikers and 2 fielders from the pool of free_allies
    '''
    ind = 0
    for i in free_allies[:2]:
        striker_action = Striker(team.ball_controler, team.enemy_goal,
                                  team.blocker_enemies, team.allies)
        team.game.add_action(striker_action, i.id, False)
        ind += 1
        
    for i in free_allies[2:]:
        fielder_action = Fielder(team.ball_controler, free_allies[ind - 2], 
                                  team.blocker_enemies, team.allies)
        team.game.add_action(fielder_action, i.id, False)
        ind += 1

        
NUM_CELLS = 20
#get the cell that the loc is in see Strategy Documentation on gdrive page 2
'''
current grid numbers
1     2         11        16
2     7         12        17
3     8         13        18
4     9         14        19
5     10        15        20
'''

    

'''
goal area 
0        1000                            8000 9000
2000
4000
'''

'''
dims
        2500        4500    6500
1000
2000
4000
5000
''' 
def get_cell(loc):
    loc = loc + np.array([4500, 3000])
    if loc[0] < 2500:
        cell = 1
    elif loc[0] < 4500:
        cell = 6
    elif loc[0] < 6500:
        cell = 11
    else:
        cell = 16
        
    
    if loc[1] < 1000:
        cell += 0
    elif loc[1] < 2000:
        cell += 1
    elif loc[1] < 4000:
        cell += 2
    elif loc[1] < 5000:
        cell += 3
    else:
        cell += 4
        
    return cell