import numpy as np
import math

# heuristic for travel time
# TODO: account for current velocity
def time_to_point(robot, target_loc, speed):
  return np.linalg.norm(robot.loc - target_loc)/speed

# gives point closest to point on line defined by lpoint and l_vec
def drop_perpendicular(point, lpoint, l_vec):
    lvec_mag = np.linalg.norm(l_vec)
    if lvec_mag < 0.001:
      return lpoint
    vec_norm = l_vec/lvec_mag
    mag_cos = (point - lpoint)*vec_norm
    return lpoint + np.linalg.norm(mag_cos)*vec_norm

# wraps large values of angle so that |angle| < math.pi
def min_angle(angle):
  if (angle > math.pi):
    angle = -(2*math.pi - angle)
  if (angle < -math.pi):
    angle = -(-2*math.pi - angle)
  return angle

# rotates direction by -rot
def convert_local(direction, rot):
  norm = direction[0]*math.cos(-rot) - direction[1]*math.sin(-rot)
  tang = direction[1]*math.cos(-rot) + direction[0]*math.sin(-rot)
  return np.array([norm, tang])
  
EPS = 1E-8
  
# make |angle| < math.pi
# this is redundant with min_angle
def normalize_angle(angle):
  while 1:
    if angle > math.pi + EPS:
      angle -= 2*math.pi
    elif angle < -math.pi - EPS:
      angle += 2*math.pi
    else:
      return angle
      
      
field_length = 5500
field_heigth = 3500
blue_goal = np.array([-5000,0])
yellow_goal = np.array([5000,0])
endzone_radius = 1500

def put_in_bounds(location):
  location = put_out_of_goalie_area(location)
  location = put_in_field(location)
  return location
      
def put_in_field(location):
  if location[0] < -field_length:
    location[0] = -field_length
  if location[0] > field_length:
    location[0] = field_length
  if location[1] < -field_heigth:
    location[1] = -field_heigth
  if location[1] > field_heigth:
    location[1] = field_heigth
  return location
      
# if location is in an goalie zone a point just outside of the goalie_area is returned
def put_out_of_goalie_area(location):
  g1v = location - yellow_goal
  if g1v[0] > 0:
    return np.array([field_length, endzone_radius*np.sign(g1v[1])])
  g1vm = np.linalg.norm(g1v)
  if g1vm < endzone_radius:
    return yellow_goal + g1v * endzone_radius / g1vm
    
  g2v = location - blue_goal
  if g2v[0] < 0:
    return np.array([-field_length, endzone_radius*np.sign(g1v[1])])
  g2vm = np.linalg.norm(g2v)
  if g2vm < endzone_radius:
    return blue_goal + g2v * endzone_radius / g2vm
  return location
  
  
# TODO this is approximate at best
robot_speed = 120
def travel_distance(r1, r2):
  dist = np.linalg.norm(r1.loc - r2.loc)
  return dist# + np.sum(r1.velocity * r2.velocity) * dist/robot_speed
