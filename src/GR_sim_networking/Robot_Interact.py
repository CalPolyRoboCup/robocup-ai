import os
import sys
dirname = os.path.dirname(__file__)
sys.path.insert(0, dirname+'/..')
sys.path.insert(0, dirname)
sys.path.insert(0, dirname + "/proto")
from basic_skills.source.robot import *

#for vector math
import numpy as np

import pygame

#for netowrorking
import socket
import struct

#current system time
import time

#to import proto
#protobuf implamentations expected in directory "proto" from run dirrectory
import grSim_Commands_pb2 as grSim_Commands_pb2
import grSim_Replacement_pb2 as grSim_Replacement_pb2
import grSim_Packet_pb2 as grSim_Packet_pb2
import messages_robocup_ssl_detection_pb2 as messages_robocup_ssl_detection_pb2
import messages_robocup_ssl_wrapper_pb2 as messages_robocup_ssl_wrapper_pb2
import referee_pb2 as referee_pb2

from motor_conversions import get_motor_speeds

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

MCAST_GRP = '224.5.23.2'
MCAST_PORT = 10020

COMMAND_GRP = "127.0.0.1"
COMMAND_PORT = 20011
    
class GRsim:
  def __init__(self, max_bots_per_team):
    # robot and ball radii
    self.robot_radius = 90
    self.ball_radius = 2
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # on this port, listen ONLY to MCAST_GRP
    sock.bind((MCAST_GRP, MCAST_PORT))
    
    mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    self.max_bots_per_team = max_bots_per_team
    self.sock = sock
    self.blue_robots = [robot(True, i, self) for i in range(self.max_bots_per_team)]
    self.yellow_robots = [robot(False, i, self) for i in range(self.max_bots_per_team)]
    self.ball = ball()
    
    
    pygame.font.init() # you have to call this at the start, 
                   # if you want to use this module.
    self.font = pygame.font.SysFont('Comic Sans MS', 20)
    pygame.display.set_caption('Pysim')

    self.clock = pygame.time.Clock()
    self.last_tick = pygame.time.get_ticks()
    self.screen_res = np.array([1040, 740])
    self.field_upper_left = np.array([-6000, -4500])
    self.field_dims = np.array([12000,9000])
    self.goal_height = 1500

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
                          if size is negative the color is orange otherwise it is blue. Either way the point has radius |size|
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
      num = self.font.render(str(br.id), False, (0,0,0))
      self.screen.blit(num, position)
      self.screen.blit(rot_image, position)
    for yr in self.yellow_robots_internal:
      rot_image = pygame.transform.rotate(self.yellow_robot_image , math.degrees(yr.rot))
      position = self.convert_to_screen_position(yr.loc, rot_image.get_rect().size)
      num = self.font.render(str(yr.id), False, (0,0,0))
      self.screen.blit(num, position)
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
        pygame.draw.circle(self.screen, (155,155,0), (int(kp[0]), int(kp[1])), int(self.ball_radius*self.screen_res[0]/self.field_dims[0])*4)
    
  
  def add_action(self, action, index, is_blue):
    '''
    brief: use this method to add actions to robots so that internal and external objects are kept up to date
    params: action - action to add
            index - the index of the bot to add action to
            is_blue - whether the bot to add action to is on the blue team
    '''
    if is_blue:
      self.blue_robots[index].add_action(action)
    else:
      self.yellow_robots[index].add_action(action)
  
  
  #advance simulation first take
  #this function does not agree with current action architecture
  def step(self):
    self.sync_with_sim()
    for blue_robot in self.blue_robots:
      blue_command = self.make_command(blue_robot)
      if None == blue_command:
        continue
      self.sock.sendto(blue_command, (COMMAND_GRP, blue_robot.command_port))
    for yellow_robot in self.yellow_robots:
      yellow_command = self.make_command(yellow_robot)
      if None == yellow_command:
        continue
      self.sock.sendto(yellow_command, (COMMAND_GRP, yellow_robot.command_port))
    return self.get_state()

  #creates single vector representation of current belief of state   
  def get_state(self):
    state_blue = []
    state_yellow = []
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
    state_blue[0:4] = state_yellow[0:4]
    state_blue[4:10+6*self.max_bots_per_team] = state_yellow[10+6*self.max_bots_per_team:]
    state_blue[10+6*self.max_bots_per_team:10+12*self.max_bots_per_team] = state_yellow[4:10+6*self.max_bots_per_team]
    return state_blue, state_yellow
  #creates protobuf command packet from action vector
  def make_command(self, robot):
    actions = robot.run_action()
    if actions == None:
      return None
    packet = grSim_Packet_pb2.grSim_Packet()
    packet.commands.timestamp = time.time()
    packet.commands.isteamyellow = not robot.is_blue
    comm = bytearray()
    motor_speeds = get_motor_speeds(action[2], action[3], action[4])
    for motor_s in motor_speeds:
      comm += (motor_s).to_bytes(2, byteorder='big')
    if action[0]:
      comm += b'1'
    else:
      comm += b'0'
    comm += action[6].to_bytes(2, byteorder='big')
    return comm
  #reads the 4 cammera updates. Ignores geometry packets
  def sync_with_sim(self):  
    for i in range(4):    
      data = self.sock.recv(1024)
      packet = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket()
      try:
        packet.ParseFromString(data)
      except:
        data = self.sock.recv(1024)
        packet.ParseFromString(data)
      self.update_from_proto(packet)
  #reads a packet and updates belief to match  
  def update_from_proto(self, packet):
    frame = packet.detection
    if (len(frame.balls) != 0):
      self.ball.update(np.array([frame.balls[0].x,frame.balls[0].y]), 
        1, time_stamp = frame.frame_number)
    else:
      self.ball.update(np.array([0, 0]), 0)

    blue_observed = [0 for _ in range(self.max_bots_per_team)]
    yellow_observed = [0 for _ in range(self.max_bots_per_team)]
    #for each instance observed update as observed
    for i in range(len(frame.robots_blue)):
      robo = frame.robots_blue[i]
      self.blue_robots[robo.robot_id].update(
        np.array([robo.x, robo.y]), robo.orientation, 1, time_stamp = frame.frame_number)
      blue_observed[robo.robot_id] = 1
    for i in range(len(frame.robots_yellow)):
      robo = frame.robots_yellow[i]
      self.yellow_robots[robo.robot_id].update(
        np.array([robo.x, robo.y]), robo.orientation, 1, time_stamp = frame.frame_number)
      yellow_observed[robo.robot_id] = 1
    #for all others update as unobserved
    for i in range(self.max_bots_per_team):
      if not blue_observed[i]:
        self.blue_robots[i].update(np.array([0,0]), 0, 0)
    for i in range(self.max_bots_per_team):
      if not yellow_observed[i]:
        self.yellow_robots[i].update(np.array([0,0]), 0, 0)

#simple test code 
if __name__ == "__main__":
  max_bots_per_team = 8
  game = GRsim(max_bots_per_team)
  for i in game.blue_robots:
    i.add_action(sample_action())
  for j in range(10000):
    game.step()
    if j%200 == 0:
      game.display()
    
