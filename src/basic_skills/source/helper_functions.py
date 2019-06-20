import numpy as np
import math
import time

def drop_perpendicular(point, lpoint, l_vec):
    # gives point closest to point on line defined by lpoint and l_vec
        mag_cos = mag_with(point - lpoint, l_vec)
        return lpoint + scale_to(l_vec, mag_cos)

def min_angle(angle):
    # wraps large values of angle so that |angle| < math.pi 
    if (angle > math.pi):
        angle = angle - 2*math.pi
    if (angle < -math.pi):
        angle = angle + 2*math.pi
    return angle

def convert_local(direction, rot):
    # rotates direction by -rot
    norm = direction[0]*math.cos(-rot) - direction[1]*math.sin(-rot)
    tang = direction[1]*math.cos(-rot) + direction[0]*math.sin(-rot)
    return np.array([norm, tang])
    
def rotate_vector(vector, angle):
    # rotate a vector angle degrees counter clockwise
    return convert_local(vector, angle)

def squash(num, val = 4000):
    '''
    function that rescales a number between 0-1 with
    piecewise linear construction
    '''
    ret = num/val
    if ret > 1:
        ret = 1
    return ret

def normalize_angle(angle):  
    '''
    make |angle| < math.pi
    this is redundant with min_angle
    '''
    while 1:
        if angle > math.pi:
            angle -= 2*math.pi
        elif angle < -math.pi:
            angle += 2*math.pi
        else:
            return angle
            
            
FIELD_LENGTH = 5500
FIELD_HEIGTH = 3500
blue_goal = np.array([-5500,0])
yellow_goal = np.array([5500,0])

def put_in_bounds(location):
    '''
    returns
    location - valid location for a robot
    fixed - whether anything was changed
    '''
    location, fixed1 = put_in_field(location)
    location, fixed2 = put_out_of_goalie_area(location)
    return location, fixed1 or fixed2
     
def put_in_field(location):
    '''
    if location is outside of the field put it inside
    returns
    location (out of goalie box) 
    fixed - was anything done
    ''' 
    fixed = 0
    if location[0] < -FIELD_LENGTH:
        location[0] = -FIELD_LENGTH
        fixed = 1
    if location[0] > FIELD_LENGTH:
        location[0] = FIELD_LENGTH
        fixed = 1
    if location[1] < -FIELD_HEIGTH:
        location[1] = -FIELD_HEIGTH
        fixed = 1
    if location[1] > FIELD_HEIGTH:
        location[1] = FIELD_HEIGTH
        fixed = 1
    return location, fixed

def put_out_of_goalie_area(location):            
    '''
    if location is in an goalie zone a point just outside of the goalie_area is returned
    TODO: Switch this to rectangular goalie box per recent rules
    returns 
    location (out of goalie box) 
    fixed - was anything done
    '''
    endzone_radius = 1500
    g1v = location - yellow_goal
    g1vm = mag(g1v)
    if g1vm < endzone_radius:
        return yellow_goal + g1v * endzone_radius / g1vm, 1
    g2v = location - blue_goal
    g2vm = mag(g2v)
    if g2vm < endzone_radius:
        return blue_goal + g2v * endzone_radius / g2vm, 1
    return location, 0
    
    
def travel_time(loc1, loc2, angle = None):
    '''
    TODO this is approximate at best
    it should predict the time for a robot to get from
    its current location to a given location
    it should have the option to include a target angle
    '''
    if type(loc1) is np.ndarray:
        return dist(loc1, loc2)/5000
    return dist(loc1.loc, loc2)/5000
    '''robot_max_speed = 120
    robot_acceleration = 5
    dist = robot.loc - loc
    current_speed = mag_with(robot.velocity, dist)
    accel_to_max_time = (robot_max_speed - current_speed) / robot_acceleration
    decelerat_from_max_time = robot_max_speed / robot_acceleration
    decelerat_tangent_time = np.sqrt(mag(robot.velocity)**2 - 
                            current_speed**2) / robot_acceleration
    
    straight_time = mag(dist) / current_speed
    if (accel_to_max_time + decelerat_from_max_time < straight_time and 
                            decelerat_tangent_time < straight_time):
        return straight_time
    return (accel_to_max_time + decelerat_from_max_time +
            mag(dist) / robot_max_speed)'''

def mag_with(v1, v2):
    '''
    returns the maginitude of v1 in the direction of v2 
    negative if opposing
    '''
    magv2 = mag(v2)
    if magv2 < 0.001:
        return 0
    return np.sum(v1*v2)/magv2

def mag(npArray):
    '''
    returns the magnitude of a np.array
    '''
    return np.linalg.norm(npArray)

def dist(npArray1, npArray2):
    '''
    returns the distance between two np.array points
    '''
    return mag(npArray1 - npArray2)

def scale_to(vector, magnitude):
    '''
    scale vector to magnitued
        vector - np.array
        magnitude - number
    '''
    magv = mag(vector)
    if magv == 0:
        return vector
    return vector * magnitude / magv

def angle_to(npFrom, npTo):
    '''
    returns the angle of the vector from a point to anouther point
    The angle is measured fromt he positive x axis
    '''
    point_dir = npFrom - npTo
    return angle_of(point_dir)

def angle_of(npVect):
    '''
    returns the angle of the vector
    The angle is measured fromt he positive x axis
    '''
    return -math.atan2(npVect[1], npVect[0])

def squash(num, val = 4000):
    '''
    function that rescales a number between 0-1 with
    piecewise linear construction
    '''
    ret = num/val
    if ret > 1:
        ret = 1
    return ret