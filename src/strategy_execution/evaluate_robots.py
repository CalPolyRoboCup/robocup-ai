import numpy as np
import math
import time
import sys
import os

dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
from worst_intercept import worst_intercept
from strategy_helpers import get_cell

sys.path.insert(0, dirname+'/..')
from basic_skills.source.helper_functions import mag, squash, angle_to, min_angle, dist
from basic_skills.source.PassTo import LEAD_FACTOR
    
    
'''
brief - runs a heuristic to determine the advantage of making a pass to each robot on the team
    HEURISTIC COMPNONENTS
        shot_value              - value given to robots for being able to shoot at the enemy goal
        enemy fear_value        - value given for being far from enemies
        down_field_value        - value given for being down field
        controler_value         - constant given to the ball controler. Serves as a passing threshold.
        cell_value              - value for being in a given cell of the field

params - team - team object. Specifies allies to evaluate and enemies to consider as blockers
                 best_reciever_index - the index of the best robot to pass to in the last time step
                                                             used for sticky value
returns - values : heuristic estimates of the value of a pass to each robot in team 
                    best_reciever_index : index of the robot with the highest score (in above vector)
                                                                except the ball_controler
'''
def evaluate_robots(team, best_reciever_index):
    #stime = time.time()

    shot_value = 1500
    alignment_weight = 0.2
    enemy_fear_value_factor = 500
    enemy_fear_radius = 600
    down_field_value_weight = 0.05
    controler_value = 500
    stand_off_value = 300
    min_pass_margin = 400
    stand_off_radius = 700

    value_propagation_rate = 0.5

    # TODO: temp values. intended for yellow team
    cell_values = [0,10,0,10,0,         # enemy goal
                  20,40,60,40,20,
                  10,20,30,20,10,
                  5,10,15,10,5]         # our goal

    # assign values to having each robot control the ball
    raw_values = []
    for r in team.allies:
        extrapolated_position = r.loc + r.velocity * LEAD_FACTOR
        worst, _ = worst_intercept(extrapolated_position, team.enemy_goal, team.blocker_enemies)
        pass_angle = abs(min_angle(angle_to(team.enemy_goal, r.loc) - r.rot))
        alignment_factor = (1 - squash(pass_angle, np.pi)) * alignment_weight + 1 - alignment_weight
        shot_likelyhood = interpret_intercept(worst) * alignment_factor
        cell_value = cell_values[get_cell(r.loc) - 1]
        down_field_value = r.loc[0] * down_field_value_weight * (1 if team.is_blue else -1)
        fear_value = enemy_fear_value_factor
        for e in team.enemies:
            edist = mag(e.loc - r.loc)
            fv = enemy_fear_value_factor * squash(edist, enemy_fear_radius)
            if fv < fear_value:
                fear_value = fv
            
        controler_value = controler_value if r.id == team.ball_controler else 0
        stand_off_value = squash(dist(team.ball_controler.loc, r.loc), stand_off_radius) * stand_off_value
            
        '''if r.id == 1:
            print(shot_likelyhood * shot_value, cell_value,
                        sticky_value, down_field_value,
                        controler_value, fear_value)
        '''
        raw_values.append(shot_likelyhood * shot_value + cell_value +
                        down_field_value + controler_value + fear_value +
                        stand_off_value)
     
    # share values between connected allies. Will boost the value of
    # robots with passes to other robots, but not isolated robots.
    highest_value_reciever = -1000 #very negative number
    values = [v for v in raw_values]
    for r in team.allies:
        extrapolated_position = r.loc + r.velocity * LEAD_FACTOR
        worst, _ = worst_intercept(team.game.ball.loc, extrapolated_position, team.blocker_enemies)
        if worst is not None and abs(worst) < min_pass_margin:
            values[r.id] = 0
            continue
        pass_angle = abs(min_angle(angle_to(r.loc, team.game.ball.loc) - team.ball_controler.rot))
        alignment_factor = (1 - squash(pass_angle, np.pi)) * alignment_weight + 1 - alignment_weight
        pass_likelyhood = interpret_intercept(worst) * alignment_factor
        for other in team.allies:
            if other.id != r.id:
                worst, _ = worst_intercept(r.loc, other.loc, team.blocker_enemies)
                pass_angle = abs(min_angle(angle_to(other.loc, r.loc) - r.rot))
                alignment_factor = (1 - squash(pass_angle, np.pi)) * alignment_weight + 1 - alignment_weight
                secondary_pass_likelyhood = interpret_intercept(worst) * alignment_factor
                values[r.id] += value_propagation_rate * raw_values[other.id] * secondary_pass_likelyhood
                
        # print(r.id, pass_likelyhood, pass_angle, angle_to(r.loc, team.game.ball.loc), team.ball_controler.rot)
        
        values[r.id] *= pass_likelyhood
        
        if highest_value_reciever < values[r.id] and r.id != team.ball_controler:
            highest_value_reciever = values[r.id]
            best_reciever_index = r.id
    
    # debugging
    team.prints = []
    for v, a in zip(values, team.allies):
        if v == max(values):
            team.prints.append((a.loc, -v/50))
        else:
            team.prints.append((a.loc, v/50))

    return values, best_reciever_index

def interpret_intercept(worst, shot_margin = 1000):
    if worst is None:
        return 1
    return squash(abs(worst), shot_margin)