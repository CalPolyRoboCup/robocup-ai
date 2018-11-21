import numpy as np
import math

def time_to_point(robot, target_loc, speed):
  return np.linalg.norm(robot.loc - target_loc)/speed

def drop_perpendicular(point, lpoint, l_vec):
    lvec_mag = np.linalg.norm(l_Vec)
    vec_norm = l_vec/lvec_mag
    mag_cos = (point - lpoint)*vec_norm
    return lpoint + np.linalg.norm(mag_cos)*vec_norm

def min_angle(angle):
  if (angle > math.pi):
    angle = -(2*math.pi - angle)
  if (angle < -math.pi):
    angle = -(-2*math.pi - angle)
  return angle

def normalize_angle(angle):
  while 1:
    if angle > math.pi:
      angle -= 2*math.pi
    elif angle < -math.pi:
      angle += 2*math.pi
    else:
      return angle
