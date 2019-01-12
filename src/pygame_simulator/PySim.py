import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '..')
from basic_skills.robot import *
from basic_skills.action import *
from basic_skills.helper_functions import *

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
ball_radius = 25
    
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
    self.last_controler = False
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

    pygame.display.set_caption('Pysim')

    self.clock = pygame.time.Clock()
    self.last_tick = pygame.time.get_ticks()
    self.screen_res = np.array([1040, 740])
    self.field_upper_left = np.array([-6000, -4500])
    self.field_dims = np.array([12000,9000])
    self.goal_height = 1500

    self.font = pygame.font.SysFont("Impact", 55)
    self.screen = pygame.display.set_mode(self.screen_res, pygame.HWSURFACE, 32)
    self.field_image = pygame.image.load("..c/resources/Field.png").convert_alpha()
    field_scale = self.field_image.get_rect().size * self.screen_res / np.array([1040, 740])
    field_scale = (int(field_scale[0]), int(field_scale[1]))
    self.field_image = pygame.transform.scale(self.field_image, field_scale)
    self.blue_robot_image = pygame.image.load("../resources/BlueBot.png").convert_alpha()
    robot_scale = 2*robot_radius*self.screen_res/self.field_dims
    robot_scale = (int(robot_scale[0]), int(robot_scale[1]))
    self.blue_robot_image = pygame.transform.scale(self.blue_robot_image, robot_scale)
    self.yellow_robot_image = pygame.image.load("../resources/YellowBot.png").convert_alpha()
    self.yellow_robot_image = pygame.transform.scale(self.yellow_robot_image, robot_scale)
    
    self.time_step = 1/60
    self.starting_formation = starting_formation
    
    self.ball = ball()
    self.reset()
    
    # self.clock.tick(60)
  # def launch():
    # while 1:
        # self.Loop()
  def reset(self):
    for i in range(self.max_bots_per_team):
      self.blue_robots[i].loc = np.array(self.starting_formation[i])
      self.blue_robots[i].rot = 0
      self.yellow_robots[i].loc = - np.array(self.starting_formation[i])
      self.yellow_robots[i].rot = -np.pi
      self.blue_robots[i].velocity = np.array([0,0])
      self.yellow_robots[i].velocity = np.array([0,0])
    self.ball.loc = np.array([0,0])
    self.ball.velocity = np.array([0,0])
  def convert_to_field_position(self, loc):
    return self.field_upper_left + loc * self.field_dims / self.screen_res
  def convert_to_screen_position(self, loc, dims = [0,0]):
    return (loc - self.field_upper_left)*self.screen_res/self.field_dims - np.array(dims)/2
  def draw(self, key_points = []):
    #self.screen.fill((150,150,150))
    self.screen.blit(self.field_image,(0,0))
    for br in self.blue_robots:
      rot_image = pygame.transform.rotate(self.blue_robot_image , math.degrees(br.rot))
      position = self.convert_to_screen_position(br.loc, rot_image.get_rect().size)
      #print(position)
      self.screen.blit(rot_image, position)
    for yr in self.yellow_robots:
      rot_image = pygame.transform.rotate(self.yellow_robot_image , math.degrees(yr.rot))
      position = self.convert_to_screen_position(yr.loc, rot_image.get_rect().size)
      self.screen.blit(rot_image, position)
    position = self.convert_to_screen_position(self.ball.loc)
    pygame.draw.circle(self.screen, (255,55,0), (int(position[0]), int(position[1])), int(ball_radius*self.screen_res[0]/self.field_dims[0]))
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
    pygame.display.update()
  def update_bot(self, robot, delta_time):
    #kicker refactory period
    KICK_CD = 180
    
    #hit box for kick
    kick_length = 40
    kick_width = 70
    
    #kick force
    kick_delta_V = 1100
    
    #hit box for spinner
    spin_length = 25
    spin_width = 100
    
    #spin force when touching
    spin_lin_accel = 5
    
    #stored spin force after contact broken (leaving it equal to spin_lin_accel works well)
    spin_rot_accel = 5
    
    #how much acceleration can be stored in spin
    max_ball_spin = 4
    
    #how quickly the ball spins up (0 to 1 inclusive) larger is slower
    spin_accel_rate = .85
    
    
    
    #how fast the current robot velocity changes to the target velocity (0 to 1 inclusive) larger is slower
    accel_rate = .99
    
    #how fast the robot can move
    max_speed = 3000
    
    #how fast the robot can turn (radians)
    max_angular_speed = 2
    
    
    
    '''
    do spinner stuff
    TODO: simulate the front of the 
    '''
    ball_robot_vector = self.ball.loc - robot.loc
    robot_local_ball_loc = convert_local(ball_robot_vector, -robot.rot)
    if (robot_local_ball_loc[0] > 0 and robot_local_ball_loc[0] < robot_radius + spin_length + ball_radius
      and abs(robot_local_ball_loc[1]) < spin_width/2):
      
      #self.ball.spin = self.ball.spin - ball_robot_vector/np.linalg.norm(ball_robot_vector) * spin_rot_accel
      self.ball.controler = robot
      pull_vel = robot.loc + convert_local([robot_radius + ball_radius, 0], robot.rot) - self.ball.loc
      pull_vel = pull_vel / np.linalg.norm(pull_vel) * max_ball_spin
      #print(pull_vel)
      self.ball.spin = self.ball.spin * spin_accel_rate + pull_vel * (1-spin_accel_rate)
      self.ball.velocity = self.ball.velocity * spin_accel_rate + pull_vel * (1-spin_accel_rate)
      #print("spin - ",robot.id, robot.is_blue, " - ", pull_vel, self.ball.velocity)
      
    '''
    handle the action on this robot
    '''
    action = robot.run_action()
    if action == None:
      action = [0,0,0,0,0]
      
    '''
    do kick stuff
    '''
    if robot.kick_cooldown > 0:
      robot.kick_cooldown -= 1
    if action[0] >= 1 and robot.kick_cooldown == 0:
      robot.kick_cooldown = KICK_CD
      #print(robot_local_ball_loc[0] > 0, robot_local_ball_loc[0] < robot_radius + kick_length + ball_radius, abs(robot_local_ball_loc[1]) > kick_width/2)
      if (robot_local_ball_loc[0] > 0 and robot_local_ball_loc[0] < robot_radius + kick_length + ball_radius
        and abs(robot_local_ball_loc[1]) < kick_width/2):
        self.ball.velocity = self.ball.velocity + kick_delta_V * ball_robot_vector / np.linalg.norm(ball_robot_vector)
      
    '''
    handle velocity inputs from wheels
    '''
    #15.3846 is there to make control magnitudes similar to GRsim
    robot.velocity = robot.velocity*accel_rate + convert_local(np.array(action[2:4])*15.3846, robot.rot) * (1-accel_rate)
    robot.rot_vel = robot.rot_vel*accel_rate + action[4] * (1-accel_rate)
    
    '''
    limit max speed
    '''
    if np.linalg.norm(robot.velocity) > max_speed:
      robot.velocity = robot.velocity * max_speed/np.linalg.norm(robot.velocity)
    if robot.rot_vel > max_angular_speed:
      robot.rot_vel = max_angular_speed
    if robot.rot_vel < -max_angular_speed:
      robot.rot_vel = -max_angular_speed
    
    robot.loc = robot.loc + robot.velocity * delta_time
    robot.rot = robot.rot + robot.rot_vel * delta_time
    return 0
  def do_collision(self, delta_time):
    ball_mass = 1
    elasticity_factor = .8
    
    robots = []
    robots.extend(self.blue_robots)
    robots.extend(self.yellow_robots)
    i = 0
    ball_held = False
    for r in robots:
      for o in robots[i+1:]:
        distance = np.linalg.norm(r.loc - o.loc)
        if distance < 2 * robot_radius:
          push_out_vector = (r.loc - o.loc) * (distance - 2*robot_radius)/(distance * 2)
          r.loc = r.loc - push_out_vector
          r.velocity = r.velocity - push_out_vector / delta_time
          o.loc = o.loc + push_out_vector
          o.velocity = o.velocity + push_out_vector / delta_time
      ball_vector = self.ball.loc - r.loc
      ball_distance = np.linalg.norm(ball_vector)
      if ball_distance < robot_radius + ball_radius:
        push_out_vector = ball_vector * ((robot_radius + ball_radius) - ball_distance)/ball_distance
        bounce_velocity = drop_perpendicular(self.ball.velocity, np.array([0,0]), ball_vector) * elasticity_factor
        if ball_held:
          r.loc = r.loc - push_out_vector
          self.ball.velocity = self.ball.velocity + push_out_vector / delta_time / ball_mass + bounce_velocity
        else:
          ball_held = True
          self.ball.velocity = self.ball.velocity + push_out_vector / delta_time / ball_mass + bounce_velocity
          self.ball.loc = self.ball.loc + push_out_vector
      i += 1
  def get_reward(self):
    if self.ball.controler != False:
      self.ball.last_controler = self.ball.controler
    if self.ball.last_controler != False:
      if (self.ball.loc[0] > (self.field_dims + self.field_upper_left)[0] or self.ball.loc[1] > (self.field_dims + self.field_upper_left)[1] or
        self.ball.loc[0] < self.field_upper_left[0] or self.ball.loc[1] < self.field_upper_left[1]):
        if (abs(self.ball.loc[1]) < self.goal_height/2):
          if self.ball.last_controler.is_blue:
            #goal for yellow
            print("goal on blue")
            reward = -50
          else:
            #goal for blue
            print("goal on yellow")
            reward = 50
        elif self.ball.last_controler.is_blue:
          #out of bounds on blue
          print("out on blue")
          reward = -5
        else:
          #out of bounds on yellow
          print("out on yellow")
          reward = 5
        return reward, True
    return self.ball.loc[0]/1000, False
  def step(self, delta_time = .01666666, key_points = []):
    spin_degen = .75
    ball_friction_factor = .999
    
    self.ball.controler = False
    
    for blue_robot in self.blue_robots:
      self.update_bot(blue_robot, delta_time)
    for yellow_robot in self.yellow_robots:
      self.update_bot(yellow_robot, delta_time)
    
    self.do_collision(delta_time)
    
    # if (np.linalg.norm(self.ball.spin) > max_ball_spin):
      # self.ball.spin = self.ball.spin * max_ball_spin / np.linalg.norm(self.ball.spin)
    if not self.ball.controler:
      self.ball.velocity = self.ball.velocity + self.ball.spin
      self.ball.spin = self.ball.spin * spin_degen
    self.ball.loc = self.ball.loc + self.ball.velocity * delta_time
    self.ball.velocity = self.ball.velocity * ball_friction_factor
    self.draw(key_points)
    blue_reward, transition = self.get_reward()
    if transition:
      self.reset()
    return self.get_state(), blue_reward, transition
  def push_state(self):
    pass
  def add_action(self, action, index, is_blue):
    if is_blue:
      self.blue_robots[index].add_action(action)
    else:
      self.yellow_robots[index].add_action(action)
  def get_state(self):
    rot_noise = .1
    vel_noise = 75
    loc_noise = 50
    state_blue = []
    state_yellow = []
    state_yellow.append(self.ball.loc[0] + (np.random.random_sample() - 0.5) * loc_noise)
    state_yellow.append(self.ball.loc[1] + (np.random.random_sample() - 0.5) * loc_noise)
    state_yellow.append(self.ball.velocity[0] + (np.random.random_sample() - 0.5) * vel_noise)
    state_yellow.append(self.ball.velocity[1] + (np.random.random_sample() - 0.5) * vel_noise)
    for YRobot in self.yellow_robots:
      state_yellow.append(YRobot.loc[0] + (np.random.random_sample() - 0.5) * loc_noise)
      state_yellow.append(YRobot.loc[1] + (np.random.random_sample() - 0.5) * loc_noise)
      state_yellow.append(YRobot.velocity[0] + (np.random.random_sample() - 0.5) * vel_noise)
      state_yellow.append(YRobot.velocity[1] + (np.random.random_sample() - 0.5) * vel_noise)
      state_yellow.append(YRobot.rot_vel + (np.random.random_sample() - 0.5) * rot_noise)
      state_yellow.append(YRobot.rot + (np.random.random_sample() - 0.5) * rot_noise)
    for BRobot in self.blue_robots:
      state_yellow.append(BRobot.loc[0] + (np.random.random_sample() - 0.5) * loc_noise)
      state_yellow.append(BRobot.loc[1] + (np.random.random_sample() - 0.5) * loc_noise)
      state_yellow.append(BRobot.velocity[0] + (np.random.random_sample() - 0.5) * vel_noise)
      state_yellow.append(BRobot.velocity[1] + (np.random.random_sample() - 0.5) * vel_noise)
      state_yellow.append(BRobot.rot_vel + (np.random.random_sample() - 0.5) * rot_noise)
      state_yellow.append(BRobot.rot + (np.random.random_sample() - 0.5) * rot_noise)
    state_blue[0:4] = state_yellow[0:4]
    state_blue[4:10+6*self.max_bots_per_team] = state_yellow[10+6*self.max_bots_per_team:]
    state_blue[10+6*self.max_bots_per_team:10+12*self.max_bots_per_team] = state_yellow[4:10+6*self.max_bots_per_team]
    return state_blue, state_yellow
    
class keyboard_control(action):
  def __init__(self):
    action.__init__(self)
    self.norm_vel = 0
    self.tang_vel = 0
    self.rot_vel = 0
    self.kick = 0
    self.speed = 65
    self.rot_speed = 4
  def keypress_update(self, keys):
    self.norm_vel = 0
    self.tang_vel = 0
    self.rot_vel = 0
    self.kick = 0
    if keys[K_SPACE]:
      self.kick = 1
    if keys[K_d]:
      self.norm_vel = -self.speed
    elif keys[K_a]:
      self.norm_vel = self.speed
    if keys[K_w]:
      self.tang_vel = self.speed
    elif keys[K_s]:
      self.tang_vel = -self.speed
    if keys[K_q]:
      self.rot_vel = self.rot_speed
    elif keys[K_e]:
      self.rot_vel = -self.rot_speed
  def run(self):
    action = [self.kick,0,self.norm_vel, self.tang_vel, self.rot_vel]
    self.action = action
    return action
    
    
#simple test code 
if __name__ == "__main__":
  max_bots_per_team = 6
  game = PYsim(max_bots_per_team)
  key_action = keyboard_control()
  clock = pygame.time.Clock()
  clock.tick(60)
  ttime = clock.tick()
  for b in game.yellow_robots:
    b.add_action(key_action)
  while 1:
    for event in pygame.event.get():
      if event.type == QUIT:
        pygame.quit()
        sys.exit()
      if event.type == KEYDOWN or event.type == KEYUP:
        keys = pygame.key.get_pressed()
        key_action.keypress_update(keys)
    new_time = clock.tick()
    #visualization test
    #game.step(key_points = [([0,1000],10), ([0,-1000],-10)])
    game.step()
    ttime = new_time
    
