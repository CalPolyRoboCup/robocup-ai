import numpy as np
import math
import sys
import os
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname)
sys.path.insert(0, dirname+"/../..")
import basic_skills
from helper_functions import angle_of, mag, rotate_vector

def sign_threshold_subtract(val, deduction):
    if val > 0:
        val -= deduction
        if val < 0:
            val = 1
    else:
        val += deduction
        if val > 0:
            val = -1
    return val

'''
brief: finds the enemy best able to intercept a pass from location to p
params:
    location        - origin of pass
    target          - target of pass
    enemies         - list of enemies ([np.array,...], or [robot,...])
    preference_id   - id to prefer by up to sticky_factor. if None ignored
returns:
    worst       - the shortest distance an enemy would need to move to 
                    block a shot from location to p. None if no enemy can
                    block the shot
'''
def worst_intercept(location, target, enemies, preference_id = None):
    ball_speed = 1000
    sticky_factor = 500
    worst = None
    first = True
    pass_vect = location - target
    pangle = -angle_of(pass_vect)
    pass_length = mag(pass_vect)
    i = 0
    worst_id = None
    for e in enemies:
        '''
        rotate enemy position into a coordinate space with the
        pass from location to target aligned to the +X axis
        '''
        if type(e) is basic_skills.source.robot.robot:
            local = rotate_vector(location - e.loc, pangle)
        else:
            # assume e is np.array
            local = rotate_vector(location - e, pangle)

        '''    
        if the enemy is in range to intercept
        track worst intercept
        '''
        if local[0] > 0 and local[0] < pass_length:

            # scale down intercept_dist
            intercept_dist = local[1]
            sign_threshold_subtract(intercept_dist, local[0] / ball_speed)
            if i == preference_id:
                sign_threshold_subtract(intercept_dist, sticky_factor)
                        
            if first or abs(intercept_dist) < abs(worst):
                worst = intercept_dist
                worst_id = i
                first = False

        i += 1
        
    return worst, worst_id