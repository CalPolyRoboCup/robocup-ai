import numpy as np
import math

#heuristic for travel time
#TODO: account for current velocity
def time_to_point(robot, target_loc, speed):
  return np.linalg.norm(robot.loc - target_loc)/speed

#gives point closest to point on line defined by lpoint and l_vec
def drop_perpendicular(point, lpoint, l_vec):
    lvec_mag = np.linalg.norm(l_vec)
    vec_norm = l_vec/lvec_mag
    mag_cos = (point - lpoint)*vec_norm
    return lpoint + np.linalg.norm(mag_cos)*vec_norm

#wraps large values of angle so that |angle| < math.pi
def min_angle(angle):
  if (angle > math.pi):
    angle = -(2*math.pi - angle)
  if (angle < -math.pi):
    angle = -(-2*math.pi - angle)
  return angle

#rotates direction by -rot
def convert_local(direction, rot):
  norm = direction[0]*math.cos(-rot) - direction[1]*math.sin(-rot)
  tang = direction[1]*math.cos(-rot) + direction[0]*math.sin(-rot)
  return np.array([norm, tang])
  
#make |angle| < math.pi
#this is redundant with min_angle
def normalize_angle(angle):
  while 1:
    if angle > math.pi:
      angle -= 2*math.pi
    elif angle < -math.pi:
      angle += 2*math.pi
    else:
      return angle
      
#if location is in an end zone a point just outside of the end zone is returned
def out_of_endzone(location):
  goal1 = np.array([-5000,0])
  goal2 = np.array([5000,0])
  endzone_radius = 1500
  g1v = location - goal1
  g1vm = np.linalg.norm(g1v)
  if g1vm < endzone_radius:
    return goal1 + g1v * endzone_radius / g1vm
  g2v = location - goal2
  g2vm = np.linalg.norm(g1v)
  if g2vm < endzone_radius:
    return goal2 + g2v * endzone_radius / g2vm
  return location
