import os
import sys
import time
#replace this with your path to robocup-ai
sys.path.insert(0, '..')
from basic_skills.robot import *
from basic_skills.action import *
from basic_skills.helper_functions import *
from pygame_simulator.ball import ball

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
allie_robots    x, y, orientation
enemy_robots    x, y, orientation
'''
    
import pygame

from pygame.locals import *

default_formation = [[-1000,0],[-2000, 500], [-2000, -500], [-3000, 1000], [-3000, -1000], [-4500, 0]]
    
class PYsim:
  def __init__(self, max_bots_per_team, starting_formation = default_formation):
  
    #robot and ball radii
    self.robot_radius = 90
    self.ball_radius = 25
  
    self.max_bots_per_team = max_bots_per_team
    self.blue_robots = [robot(True, i, self) for i in range(self.max_bots_per_team)]
    self.yellow_robots = [robot(False, i, self) for i in range(self.max_bots_per_team)]
    
    self.blue_robots_internal = [robot(True, i, self) for i in range(self.max_bots_per_team)]
    self.yellow_robots_internal = [robot(False, i, self) for i in range(self.max_bots_per_team)]

    pygame.display.set_caption('Pysim')

    self.clock = pygame.time.Clock()
    self.last_tick = pygame.time.get_ticks()
    self.screen_res = np.array([1040, 740])
    self.field_upper_left = np.array([-6000, -4500])
    self.field_dims = np.array([12000,9000])
    self.goal_height = 1500

    self.font = pygame.font.SysFont("Impact", 55)
    self.screen = pygame.display.set_mode(self.screen_res, pygame.HWSURFACE, 32)
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, "../resources/Field.png")
    self.field_image = pygame.image.load(filename).convert_alpha()
    field_scale = self.field_image.get_rect().size * self.screen_res / np.array([1040, 740])
    field_scale = (int(field_scale[0]), int(field_scale[1]))
    self.field_image = pygame.transform.scale(self.field_image, field_scale)
    
    filename = os.path.join(dirname, "../resources/BlueBot.png")
    self.blue_robot_image = pygame.image.load(filename).convert_alpha()
    robot_scale = 2*self.robot_radius*self.screen_res/self.field_dims
    robot_scale = (int(robot_scale[0]), int(robot_scale[1]))
    self.blue_robot_image = pygame.transform.scale(self.blue_robot_image, robot_scale)
    
    filename = os.path.join(dirname, "../resources/YellowBot.png")
    self.yellow_robot_image = pygame.image.load(filename).convert_alpha()
    self.yellow_robot_image = pygame.transform.scale(self.yellow_robot_image, robot_scale)
    
    self.time_step = 1/60
    self.starting_formation = starting_formation
    
    self.ball = ball()
    self.ball_internal = ball()
    
    '''
    physics parameters
    '''

    #kicker refactory period
    self.kick_cd = 180
    
    #hit box for kick
    self.kick_length = 40
    self.kick_width = 70
    
    #kick force
    self.kick_delta_V = 1100
    
    #hit box for spinner
    self.spin_length = 25
    self.spin_width = 100
    
    #spin force when touching
    self.spin_lin_accel = 5
    
    #stored spin force after contact broken (leaving it equal to spin_lin_accel works well)
    self.spin_rot_accel = 5
    
    #how much acceleration can be stored in spin
    self.max_ball_spin = 4
    
    #how quickly the ball spins up (0 to 1 inclusive) larger is slower
    self.spin_accel_rate = .85
    
    #how fast the current robot velocity changes to the target velocity (0 to 1 inclusive) larger is slower
    self.accel_rate = .99
    
    #how fast the robot can move
    self.max_speed = 3000
    
    #how fast the robot can turn (radians)
    self.max_angular_speed = 2
    
    #mass of the ball affects collision push out speed
    self.ball_mass = 1
    
    #1 perfectly elastic (bounces)
    #0 perfectly inelastic (sticks)
    self.elasticity_factor = .5
    
    #how quickly the ball looses spin
    self.spin_degen = .75
    
    #how quickly the ball slows down when not controlled (larger slows down less)
    self.ball_friction_factor = .999
    
    #noise parameterization
    self.translation_noise = 0
    self.rotation_noise = 0
    self.vanish_prob = 0
    
    self.reset()
    
  def reset(self):
    '''
      brief: place all robots in starting fomation and the ball in the center
    '''
    for i in range(self.max_bots_per_team):
      self.blue_robots_internal[i].loc = np.array(self.starting_formation[i])
      self.blue_robots_internal[i].rot = 0
      self.yellow_robots_internal[i].loc = - np.array(self.starting_formation[i])
      self.yellow_robots_internal[i].rot = -np.pi
      self.blue_robots_internal[i].velocity = np.array([0,0])
      self.yellow_robots_internal[i].velocity = np.array([0,0])
    self.ball_internal.loc = np.array([0,0])
    self.ball_internal.velocity = np.array([0,0])
    
  def convert_to_field_position(self, loc):
    '''
    brief - convert a screen pos to a field location
    param - loc: a screen position
    return - a field location
    '''
    return self.field_upper_left + loc * self.field_dims / self.screen_res
    
  def convert_to_screen_position(self, loc, dims = [0,0]):
    '''
    brief - convert a screen pos to a field location
    param - loc: a screen position
    return - a field location
    '''
    return (loc - self.field_upper_left)*self.screen_res/self.field_dims - np.array(dims)/2
    
  def draw(self, key_points = []):
    '''
    brief: render the game
    params: key_points - points to plot for debugging. A list of values. Values can be a [x,y] point or a tuple of ([x,y], size).
                          if size is negative the color is orange 
    '''
    self.render_game()
    self.render_key_points(key_points)
    pygame.display.update()
    
  def render_game(self):
    '''
    render the game
    '''
    self.screen.blit(self.field_image,(0,0))
    for br in self.blue_robots_internal:
      rot_image = pygame.transform.rotate(self.blue_robot_image , math.degrees(br.rot))
      position = self.convert_to_screen_position(br.loc, rot_image.get_rect().size)
      #print(position)
      self.screen.blit(rot_image, position)
    for yr in self.yellow_robots_internal:
      rot_image = pygame.transform.rotate(self.yellow_robot_image , math.degrees(yr.rot))
      position = self.convert_to_screen_position(yr.loc, rot_image.get_rect().size)
      self.screen.blit(rot_image, position)
    position = self.convert_to_screen_position(self.ball_internal.loc)
    pygame.draw.circle(self.screen, (255,55,0), (int(position[0]), int(position[1])), int(self.ball_radius*self.screen_res[0]/self.field_dims[0]))
    
  def render_key_points(self, key_points):
    '''
    render the key points
    '''
    for kp in key_points:
      if type(kp) is tuple:
        plot_point = self.convert_to_screen_position(np.array(kp[0]))
        scale = int(kp[1])
        if scale < 0:
          pygame.draw.circle(self.screen, (155,155,0), (int(plot_point[0]), int(plot_point[1])), -scale)
        else:
          pygame.draw.circle(self.screen, (0,155,155), (int(plot_point[0]), int(plot_point[1])), scale)
      else:
        kp = self.convert_to_screen_position(np.array(kp))
        pygame.draw.circle(self.screen, (155,155,0), (int(kp[0]), int(kp[1])), int(ball_radius*self.screen_res[0]/self.field_dims[0])*4)
    
  def update_bot_spinner(self, robot, delta_time):
    '''
    spinner stuff
    '''
    ball_robot_vector = self.ball_internal.loc - robot.loc
    robot_local_ball_loc = convert_local(ball_robot_vector, -robot.rot)
    if (robot_local_ball_loc[0] > 0 and robot_local_ball_loc[0] < self.robot_radius + self.spin_length + self.ball_radius
      and abs(robot_local_ball_loc[1]) < self.spin_width/2):
      
      #self.ball_internal.spin = self.ball_internal.spin - ball_robot_vector/np.linalg.norm(ball_robot_vector) * spin_rot_accel
      self.ball_internal.controler = robot
      pull_vel = robot.loc + convert_local([self.robot_radius + self.ball_radius, 0], robot.rot) - self.ball_internal.loc
      pull_vel = pull_vel / np.linalg.norm(pull_vel) * self.max_ball_spin
      #print(pull_vel)
      self.ball_internal.spin = self.ball_internal.spin * self.spin_accel_rate + pull_vel * (1-self.spin_accel_rate)
      self.ball_internal.velocity = self.ball_internal.velocity * self.spin_accel_rate + pull_vel * (1-self.spin_accel_rate)
      #print("spin - ",robot.id, robot.is_blue, " - ", pull_vel, self.ball_internal.velocity)
    
  def update_bot_kicker(self, robot, action):
    '''
    brief: updates a robot's kicker
    params: robot - robot to update 
            action - the action taken by the robot
    '''
    ball_robot_vector = self.ball_internal.loc - robot.loc
    robot_local_ball_loc = convert_local(ball_robot_vector, -robot.rot)
    if robot.kick_cooldown > 0:
      robot.kick_cooldown -= 1
    if action[0] and robot.kick_cooldown == 0:
      #print("kick")
      robot.kick_cooldown = self.kick_cd
      #print(robot_local_ball_loc[0] > 0, robot_local_ball_loc[0] < robot_radius + kick_length + ball_radius, abs(robot_local_ball_loc[1]) > kick_width/2)
      if (robot_local_ball_loc[0] > 0 and robot_local_ball_loc[0] < self.robot_radius + self.kick_length + self.ball_radius
        and abs(robot_local_ball_loc[1]) < self.kick_width/2):
        self.ball_internal.velocity = self.ball_internal.velocity + self.kick_delta_V * np.array([np.cos(-robot.rot), np.sin(-robot.rot)])
    
  def update_bot_movement(self, robot, action, delta_time):
    '''
    brief: updates a robot's movement
    params: robot - robot to update 
            action - the action taken by the robot
            delta_time - time since last update
    '''
    
    #15.3846 is there to make control magnitudes similar to GRsim
    robot.velocity = robot.velocity*self.accel_rate + convert_local(np.array(action[2:4])*15.3846, robot.rot) * (1-self.accel_rate)
    robot.rot_vel = robot.rot_vel*self.accel_rate + action[4] * (1-self.accel_rate)
    
    '''
    limit max speed
    '''
    if np.linalg.norm(robot.velocity) > self.max_speed:
      robot.velocity = robot.velocity * max_speed/np.linalg.norm(robot.velocity)
    if robot.rot_vel > self.max_angular_speed:
      robot.rot_vel = self.max_angular_speed
    if robot.rot_vel < -self.max_angular_speed:
      robot.rot_vel = -self.max_angular_speed
    
    robot.loc = robot.loc + robot.velocity * delta_time
    robot.rot = robot.rot + robot.rot_vel * delta_time
    
  def update_bot(self, robot, delta_time):
    '''
    brief - updates robot physics for a delta_time time step
    params - robot: the robot to update
            delta_time: the time between this step and the previous update
    '''
    self.update_bot_spinner(robot, delta_time)
    action = robot.run_action()
    if action == None:
      action = [0,0,0,0,0]
    self.update_bot_kicker(robot, action)
    self.update_bot_movement(robot, action, delta_time)
    
  def do_ball_collision(self, robot, ball_held, delta_time):
    '''
    brief: handle collisions with balls
    params: robot - the robot to do collision for
            ball_held - whether the ball has already been pushed out of a robot
    return: ball_held - an updated version of ball_held
    '''
    ball_vector = self.ball_internal.loc - robot.loc
    ball_distance = np.linalg.norm(ball_vector)
    if ball_distance < self.robot_radius + self.ball_radius:
      push_out_vector = ball_vector * ((self.robot_radius + self.ball_radius) - ball_distance)/ball_distance
      bounce_velocity = drop_perpendicular(self.ball_internal.velocity, np.array([0,0]), ball_vector) * self.elasticity_factor
      '''
      only move the ball unless anouther robot is pushing on the ball already
      '''
      if ball_held:
        robot.loc = robot.loc - push_out_vector
        self.ball_internal.velocity = self.ball_internal.velocity + push_out_vector / delta_time / self.ball_mass + bounce_velocity
      else:
        ball_held = True
        self.ball_internal.velocity = self.ball_internal.velocity + push_out_vector / delta_time / self.ball_mass + bounce_velocity
        self.ball_internal.loc = self.ball_internal.loc + push_out_vector
    return ball_held
    
  def do_robot_collision(self, r, robots, delta_time):
    '''
    brief: handle collisions with other robots
    params: r - robot to run collision on
    '''
    for o in robots:
      distance = np.linalg.norm(r.loc - o.loc)
      if distance < 2 * self.robot_radius:
        push_out_vector = (r.loc - o.loc) * (distance - 2*self.robot_radius)/(distance * 2)
        r.loc = r.loc - push_out_vector
        r.velocity = r.velocity - push_out_vector / delta_time
        o.loc = o.loc + push_out_vector
        o.velocity = o.velocity + push_out_vector / delta_time
    
  def do_collision(self, delta_time):
    '''
    brief: do all collisions
    params: delta_time - time since last update
    '''
    robots = []
    robots.extend(self.blue_robots_internal)
    robots.extend(self.yellow_robots_internal)
    ball_held = False
    for r in robots:
      robots.remove(r)
      self.do_robot_collision(r, robots, delta_time)
      ball_held = self.do_ball_collision(r, ball_held, delta_time)
      
  def get_reward(self):
    '''
    brief: checks for out of bounds and goals
    returns - reward: the reward for the blue team in this state
              done: True if a goal is made or a foul is committed
    '''
    if self.ball_internal.controler != False:
      self.ball_internal.last_controler = self.ball_internal.controler
    if self.ball_internal.last_controler != False:
      if (self.ball_internal.loc[0] > (self.field_dims + self.field_upper_left)[0] or self.ball_internal.loc[1] > (self.field_dims + self.field_upper_left)[1] or
        self.ball_internal.loc[0] < self.field_upper_left[0] or self.ball_internal.loc[1] < self.field_upper_left[1]):
        if (abs(self.ball_internal.loc[1]) < self.goal_height/2):
          if self.ball_internal.last_controler.is_blue:
            #goal for yellow
            print("goal on blue")
            reward = -50
          else:
            #goal for blue
            print("goal on yellow")
            reward = 50
        elif self.ball_internal.last_controler.is_blue:
          #out of bounds on blue
          print("out on blue")
          reward = -5
        else:
          #out of bounds on yellow
          print("out on yellow")
          reward = 5
        return reward, True
    return self.ball_internal.loc[0]/1000, False
    
  def step(self, delta_time = .01666666, key_points = []):
    state = self.get_state()
    self.ball_internal.controler = False
    
    for blue_robot in self.blue_robots_internal:
      self.update_bot(blue_robot, delta_time)
    for yellow_robot in self.yellow_robots_internal:
      self.update_bot(yellow_robot, delta_time)
    
    self.do_collision(delta_time)
    
    '''
    if the ball is free its spin will adjust its velocity slightly
    '''
    # if (np.linalg.norm(self.ball_internal.spin) > max_ball_spin):
      # self.ball_internal.spin = self.ball_internal.spin * max_ball_spin / np.linalg.norm(self.ball_internal.spin)
    if not self.ball_internal.controler:
      self.ball_internal.velocity = self.ball_internal.velocity + self.ball_internal.spin
      self.ball_internal.spin = self.ball_internal.spin * self.spin_degen
    self.ball_internal.loc = self.ball_internal.loc + self.ball_internal.velocity * delta_time
    self.ball_internal.velocity = self.ball_internal.velocity * self.ball_friction_factor
    self.draw(key_points)
    blue_reward, transition = self.get_reward()
    if transition:
      self.reset()
    return state, blue_reward, transition
    
  def push_state(self):
    pass
    
  def add_action(self, action, index, is_blue):
    #print("add act", action, index, is_blue)
    if is_blue:
      self.blue_robots_internal[index].add_action(action)
      self.blue_robots[index].add_action(action)
    else:
      self.yellow_robots_internal[index].add_action(action)
      self.yellow_robots[index].add_action(action)
      
  def get_state(self):
    '''
    brief: inject noise into ball_internal, and robot_internal states and update external objects
    returns: state_blue - state vector for blue team
             state_yellow - state vector for yellow team
    '''
    
    state_blue = []
    state_yellow = []
    state_blue[0:4] = state_yellow[0:4]
    state_blue[4:10+6*self.max_bots_per_team] = state_yellow[10+6*self.max_bots_per_team:]
    state_blue[10+6*self.max_bots_per_team:10+12*self.max_bots_per_team] = state_yellow[4:10+6*self.max_bots_per_team]
    
    self.ball.update(self.ball_internal.loc + np.random.normal(size = [2]) * self.translation_noise, np.random.randint(2))
    i = 0
    for BRobot in self.blue_robots:
      BRobot.update(self.blue_robots_internal[i].loc + np.random.normal(size = [2]) * self.translation_noise, 
        self.blue_robots_internal[i].rot + np.random.normal() * self.rotation_noise,
        np.random.choice([0,1], p = [self.vanish_prob, 1 - self.vanish_prob]))
      i += 1
    
    i = 0
    for YRobot in self.yellow_robots:
      YRobot.update(self.yellow_robots_internal[i].loc + np.random.normal(size = [2]) * self.translation_noise, 
        self.yellow_robots_internal[i].rot + np.random.normal() * self.rotation_noise, 
        np.random.choice([0,1], p = [self.vanish_prob, 1 - self.vanish_prob]))
      i += 1
    self.ball.controler = self.ball_internal.controler
    
    state_yellow.append(self.ball.loc[0])
    state_yellow.append(self.ball.loc[1])
    state_yellow.append(self.ball.velocity[0])
    state_yellow.append(self.ball.velocity[1])
    for YRobot in self.yellow_robots:
      state_yellow.append(YRobot.loc[0])
      state_yellow.append(YRobot.loc[1])
      state_yellow.append(YRobot.velocity[0])
      state_yellow.append(YRobot.velocity[1])
      state_yellow.append(YRobot.rot_vel)
      state_yellow.append(YRobot.rot)
    for BRobot in self.blue_robots:
      state_yellow.append(BRobot.loc[0])
      state_yellow.append(BRobot.loc[1])
      state_yellow.append(BRobot.velocity[0])
      state_yellow.append(BRobot.velocity[1])
      state_yellow.append(BRobot.rot_vel)
      state_yellow.append(BRobot.rot)
    
    return state_blue, state_yellow