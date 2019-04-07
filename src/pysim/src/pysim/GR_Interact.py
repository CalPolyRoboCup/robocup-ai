import sys
from basic_skills.robot import *
from basic_skills.helper_functions import *
from basic_skills.action import *

#for vector math
import numpy as np
import math

'''
GRsimGame environment documentation

inputs:
kickspeedx 0
kickspeedz 1
veltangent 2
velnormal  3
velangular 4

output:
ball           x, y, observed
blue_robots    x, y, orientation, observed
'''

#constants
#sorry about the global variables
robot_radius = 90
ball_radius = 55
    
import pygame

from pygame.locals import *

pygame.init()

class ball:
  def __init__(self):
    self.loc = np.array([0,0], dtype = np.float64)
    self.observed = 0
    self.velocity = np.array([0,0], dtype = np.float64)
    
    self.smoothing = 0
    self.first = True
    self.last_timestamp = 0
    self.spin = np.array([0,0], dtype = np.float64)
    self.controler = False
  def update(self, nloc, obs, time_elapsed = 1.0/60, time_stamp = None):
    self.observed = obs
    if (((time_stamp == None) or (time_stamp != self.last_timestamp)) and 
        obs and np.abs(nloc[0]) < 6000 and np.abs(nloc[1]) < 4000):    
      self.last_timestamp = time_stamp
      self.velocity = (self.velocity * self.smoothing + 
        (1-self.smoothing) * (nloc - self.loc)/time_elapsed)
      self.loc = ((self.loc + self.velocity*time_elapsed) * self.smoothing + 
        (1-self.smoothing) * (nloc))
    
default_formation = [[-1000,0],[-2000, 500], [-2000, -500], [-3000, 1000], [-3000, -1000], [-4500, 0]]
    
class PYsim:
  def __init__(self, max_bots_per_team, starting_formation = default_formation):
    self.max_bots_per_team = max_bots_per_team
    self.blue_robots = [robot(True, i, self) for i in range(self.max_bots_per_team)]
    self.yellow_robots = [robot(False, i, self) for i in range(self.max_bots_per_team)]
    for i in range(self.max_bots_per_team):
      self.blue_robots[i].loc = np.array(starting_formation[i])
      self.yellow_robots[i].loc = - np.array(starting_formation[i])
      self.yellow_robots[i].rot = -np.pi
    self.ball = ball()

    pygame.display.set_caption('Pysim')

    self.clock = pygame.time.Clock()
    self.last_tick = pygame.time.get_ticks()
    self.screen_res = np.array([1040, 740])
    self.field_upper_left = np.array([-6000, -4500])
    self.field_dims = np.array([12000,9000])

    self.font = pygame.font.SysFont("Impact", 55)
    self.screen = pygame.display.set_mode(self.screen_res, pygame.HWSURFACE, 32)
    self.field_image = pygame.image.load("../resources/Field.png").convert_alpha()
    self.blue_robot_image = pygame.image.load("../resources/BlueBot.png").convert_alpha()
    self.yellow_robot_image = pygame.image.load("../resources/YellowBot.png").convert_alpha()
    
    self.time_step = 1/60
    
    # self.clock.tick(60)
  # def launch():
    # while 1:
        # self.Loop()
  def convert_to_screen_position(self, loc, dims = [0,0]):
    return (loc - self.field_upper_left)*self.screen_res/self.field_dims - np.array(dims)/2
  def draw(self):
    self.screen.fill((150,150,150))
    self.screen.blit(self.field_image,(0,0))
    for br in self.blue_robots:
      rot_image = pygame.transform.rotate(self.blue_robot_image , math.degrees(br.rot))
      position = self.convert_to_screen_position(br.loc, rot_image.get_rect().size)
      self.screen.blit(rot_image, position)
    for yr in self.yellow_robots:
      rot_image = pygame.transform.rotate(self.yellow_robot_image , math.degrees(yr.rot))
      position = self.convert_to_screen_position(yr.loc, rot_image.get_rect().size)
      self.screen.blit(rot_image, position)
    position = self.convert_to_screen_position(self.ball.loc)
    print(ball_radius, position)
    pygame.draw.circle(self.screen, (255,55,0), (int(position[0]), int(position[1])), int(ball_radius*self.screen_res[0]/self.field_dims[0]))
    pygame.display.update()
  def update_bot(self, robot):
    KICK_CD = 60
    kick_length = 30
    kick_width = 20
    kick_delta_V = 500
    spin_length = 15
    spin_width = 50
    spin_lin_accel = 5
    spin_rot_accel = 5
    max_accel = 15 #dimension wise
    max_speed = 1000
    max_angular_accel = 1
    max_angular_speed = 2
    
    action = robot.run_action()
    if action == None:
      return 1
    
    ball_robot_vector = self.ball.loc - robot.loc
    robot_local_ball_loc = convert_local(ball_robot_vector, robot.rot)
    if (robot_local_ball_loc[0] > 0 and robot_local_ball_loc[0] < robot_radius + spin_length
      and abs(robot_local_ball_loc[1]) > spin_width/2):
      self.ball.spin = self.ball.spin - ball_robot_vector/np.linalg.norm(ball_robot_vector) * spin_rot_accel
      self.ball.controler = robot
      pull_vel = robot.loc + convert_local([robot_radius + ball_radius, 0], -robot.rot) - self.ball.loc
      self.ball.velocity = self.ball.velocity + pull_vel / np.linalg.norm(pull_vel) * spin_lin_accel
      
    if robot.kick_cooldown >= 0:
      robot.kick_cooldown -= 1
      
    if action[0] >= 1 and robot.kick_cooldown == 0:
      robot.kick_cooldown = KICK_CD
      if (robot_local_ball_loc[0] > 0 and robot_local_ball_loc[0] < robot_radius + kick_length
        and abs(robot_local_ball_loc[1]) > kick_width/2):
        self.ball.velocity = self.ball.velocity - kick_delta_V * ball_robot_vector / np.linalg.norm(ball_robot_vector)
    #ignore chip for now
    if action[2] > max_accel:
      action[2] = max_accel
    if action[2] < -max_accel:
      action[2] = -max_accel
    if action[3] > max_accel:
      action[3] = max_accel
    if action[3] < -max_accel:
      action[3] = -max_accel
    if action[4] > max_angular_accel:
      action[4] = max_angular_accel
    if action[4] < -max_angular_accel:
      action[4] = -max_angular_accel
      
    robot.velocity = robot.velocity + action[2:4]
    robot.rot_vel = robot.rot_vel + action[4]
    
    if np.linalg.norm(robot.velocity) > max_speed:
      robot.velocity = robot.velocity * max_speed/np.linalg.norm(robot.velocity)
    if robot.rot_vel > max_angular_speed:
      robot.rot_vel = max_angular_speed
    if robot.rot_vel < -max_angular_speed:
      robot.rot_vel = -max_angular_speed
    
    robot.loc = robot.loc + robot.velocity
    robot.rot = robot.rot + robot.rot_vel
    return 0
  def do_collision(self):
    robots = []
    robots.extend(self.blue_robots)
    robots.extend(self.yellow_robots)
    i = 0
    for r in robots:
      for o in robots[i+1:]:
        distance = np.linalg.norm(r.loc - o.loc)
        if distance < 2 * robot_radius:
          push_out_vector = (r.loc - o.loc) * (distance - 2*robot_radius)/(distance * 2)
          r.loc = r.loc - push_out_vector
          r.velocity = r.velocity - push_out_vector / self.time_step
          o.loc = o.loc + push_out_vector
          o.velocity = o.velocity + push_out_vector / self.time_step
      ball_vector = self.ball.loc - r.loc
      ball_distance = np.linalg.norm(ball_vector)
      if ball_distance < robot_radius + ball_radius:
        push_out_vector = ball_vector * (ball_distance - robot_radius + ball_radius)/ball_distance
        ball.loc = ball.loc +push_out_vector
        ball.velocity = ball.velocity +push_out_vector / self.time_stephgf
      i += 1
  def step(self):
    max_ball_spin = 20
    spin_degen = .99
    for blue_robot in self.blue_robots:
      self.update_bot(blue_robot)
    for yellow_robot in self.yellow_robots:
      self.update_bot(blue_robot)
    
    self.do_collision()
    
    if (np.linalg.norm(self.ball.spin) > max_ball_spin):
      self.ball.spin = self.ball.spin * max_ball_spin / np.linalg.norm(self.ball.spin)
    if not self.ball.controler:
      self.ball.velocity = self.ball.velocity + self.ball.spin
      self.ball.spin = self.ball.spin * spin_degen
    self.ball.loc = self.ball.loc + self.ball.velocity
    self.ball.controler = False
    self.draw()
    return self.get_state()
  def get_state(self):
    rot_noise = .1
    vel_noise = 75
    loc_noise = 50
    state_blue = []
    state_yellow = []
    state_yellow.append(self.ball.loc[0] + (np.random.random_sample - 0.5) * loc_noise)
    state_yellow.append(self.ball.loc[1] + (np.random.random_sample - 0.5) * loc_noise)
    state_yellow.append(self.ball.velocity[0] + (np.random.random_sample - 0.5) * vel_noise)
    state_yellow.append(self.ball.velocity[1] + (np.random.random_sample - 0.5) * vel_noise)
    for YRobot in self.yellow_robots:
      state_yellow.append(YRobot.loc[0] + (np.random.random_sample - 0.5) * loc_noise)
      state_yellow.append(YRobot.loc[1] + (np.random.random_sample - 0.5) * loc_noise)
      state_yellow.append(YRobot.velocity[0] + (np.random.random_sample - 0.5) * vel_noise)
      state_yellow.append(YRobot.velocity[1] + (np.random.random_sample - 0.5) * vel_noise)
      state_yellow.append(YRobot.rot_vel + (np.random.random_sample - 0.5) * rot_noise)
      state_yellow.append(YRobot.rot + (np.random.random_sample - 0.5) * rot_noise)
    for BRobot in self.blue_robots:
      state_yellow.append(BRobot.loc[0] + (np.random.random_sample - 0.5) * loc_noise)
      state_yellow.append(BRobot.loc[1] + (np.random.random_sample - 0.5) * loc_noise)
      state_yellow.append(BRobot.velocity[0] + (np.random.random_sample - 0.5) * vel_noise)
      state_yellow.append(BRobot.velocity[1] + (np.random.random_sample - 0.5) * vel_noise)
      state_yellow.append(BRobot.rot_vel + (np.random.random_sample - 0.5) * rot_noise)
      state_yellow.append(BRobot.rot + (np.random.random_sample - 0.5) * rot_noise)
    state_blue[0:4] = state_yellow[0:4]
    state_blue[4:10+6*self.max_bots_per_team] = state_yellow[10+6*self.max_bots_per_team:]
    state_blue[10+6*self.max_bots_per_team:10+12*self.max_bots_per_team] = state_yellow[4:10+6*self.max_bots_per_team]
    return state_blue, state_yellow
    
class keyboard_control(action):
  pass   
#simple test code 
if __name__ == "__main__":
  max_bots_per_team = 6
  game = PYsim(max_bots_per_team)
  for b in game.blue_robots:
    b.add_action(keyboard_control())
  for j in range(10000):
    game.step()
    if j%200 == 0:
      game.display()
    
