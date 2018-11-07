import numpy as np

import socket
import struct
import time
import math
import sys
sys.path.insert(0, './proto')

import grSim_Commands_pb2
import grSim_Replacement_pb2
import grSim_Packet_pb2
import messages_robocup_ssl_detection_pb2
import messages_robocup_ssl_wrapper_pb2
import referee_pb2
import matplotlib.pyplot as plt

'''
GRsimGame environment documentation

inputs:
kickspeedx 0
kickspeedz 1
veltangent 2
velnormal  3
velangular 4
spinner    5     --thresholds at .5

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

class ball:
  def __init__(self):
    self.loc = np.array([0,0])
    self.observed = 0
    self.velocity = np.array([0,0])
    
    self.smoothing = 0
    self.first = True
    self.last_timestamp = 0
  def update(self, nloc, obs, time_elapsed = 1.0/60, time_stamp = None):
    
    if (time_stamp == None) or (time_stamp != self.last_timestamp):
      self.observed = obs
      if obs and np.abs(nloc[0]) < 6000 and np.abs(nloc[1]) < 4000:
        if self.first:
          self.first = False
          self.velocity = np.array([0,0])
          self.loc = nloc
        else:
          #print(time_elpsed, "time")
          #print("got to", self.loc, self.velocity, nloc)
          self.last_time = time.time()
          self.velocity = (self.velocity * self.smoothing + 
            (1-self.smoothing) * (nloc - self.loc)/time_elapsed)
          self.loc = ((self.loc + self.velocity*time_elapsed) * self.smoothing + 
            (1-self.smoothing) * (nloc))
      #else:
        #print("unobs", self.velocity, self.loc, nloc)
        #self.loc = (self.loc + self.velocity*time_elapsed)

def min_angle(angle):
  while 1:
    if angle > math.pi:
      angle -= 2*math.pi
    elif angle < -math.pi:
      angle += 2*math.pi
    else:
      return angle

class robot:
  def __init__(self, is_blue, idNum):
    self.loc = np.array([0,0])
    self.rot = 0
    self.rot_vel = 0
    self.observed = 0
    self.velocity = np.array([0,0])
    self.is_blue = is_blue
    self.id = idNum
    self.last_time = time.time()
    #I don't want to make a RNN because it slows down training
    #I want to use a low pass running average filter to get 
    #velocity and feed it to a linear model maybe convolutional 
    #if we want to share weights for low level robot operations
    self.smoothing = 0.5
    self.first = True
    self.last_timestamp = 0
  def update(self, nloc, nrot, obs, time_elapsed = 1.0/60, time_stamp = None):
    #print("up")
    #print(time_stamp, self.last_timestamp, (time_stamp == None) or (time_stamp != self.last_timestamp))
    
    self.observed = obs
    if ((time_stamp == None) or (time_stamp != self.last_timestamp)) and obs and np.abs(nloc[0]) < 6000 and np.abs(nloc[1]) < 4000:
      #print("update", obs, self.id, self.is_blue, self.loc, nloc)      
      self.last_timestamp = time_stamp
      self.velocity = (self.velocity * self.smoothing + 
        (1-self.smoothing) * (nloc - self.loc)/time_elapsed)
      delta_rot = min_angle(nrot - self.rot)
      self.rot_vel = (self.rot_vel * self.smoothing + 
        (1-self.smoothing) * (delta_rot)/time_elapsed)
      self.loc = ((self.loc + self.velocity*time_elapsed) * self.smoothing + 
        (1-self.smoothing) * (nloc))
      self.rot = ((self.rot + self.rot_vel*time_elapsed) * self.smoothing + 
        (1-self.smoothing) * (nrot))
    
    

class GRsim:
  def __init__(self, max_bots_per_team):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    # on this port, listen ONLY to MCAST_GRP
    sock.bind((MCAST_GRP, MCAST_PORT))

    mreq = struct.pack("4sl", socket.inet_aton(MCAST_GRP), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    self.max_bots_per_team = max_bots_per_team
    self.sock = sock
    self.blue_robots = [robot(True, i) for i in range(self.max_bots_per_team)]
    self.yellow_robots = [robot(False, i) for i in range(self.max_bots_per_team)]
    self.ball = ball()
  def display(self, keypoints = []):
    plt.figure(1)
    
    plt.scatter([kp[0] for kp in keypoints], [kp[1] for kp in keypoints], color = "red", s = 250)

    plt.scatter([br.loc[0] for br in self.blue_robots], [br.loc[1] for br in self.blue_robots], color = 'b', s = 100)
    plt.scatter([yr.loc[0] for yr in self.yellow_robots], [yr.loc[1] for yr in self.yellow_robots], color = 'y', s = 100)
    for br in self.blue_robots:
      plt.plot([br.loc[0], br.loc[0] + 0.2*br.velocity[0]], [br.loc[1], br.loc[1] + 0.2*br.velocity[1]], color = 'black', linewidth = 3)
      plt.plot([br.loc[0] + 170*np.cos(br.rot)], [br.loc[1] + 170*np.sin(br.rot)], color = 'b', linewidth = 2)
    for yr in self.yellow_robots:
      plt.plot([yr.loc[0], yr.loc[0] + 0.2*yr.velocity[0]], [yr.loc[1], yr.loc[1] + 0.2*yr.velocity[1]], color = 'black', linewidth = 3)
      plt.plot([yr.loc[0] + 170*np.cos(yr.rot)], [yr.loc[1] + 170*np.sin(yr.rot)], color = 'yellow', linewidth = 2)
 
    plt.scatter([self.ball.loc[0]], [self.ball.loc[1]], color = 'g')
    plt.plot([self.ball.loc[0], self.ball.loc[0] + 0.2*self.ball.velocity[0]], [self.ball.loc[1], self.ball.loc[1] + 0.2*self.ball.velocity[1]], color = 'green', linewidth = 3)
    plt.show(block = False)
    plt.pause(1E-12)
    plt.clf()
  def step(self, action_blue, action_yellow):
    self.sync_with_sim()
    blue_command = self.make_command(action_blue, True)
    yellow_command = self.make_command(action_yellow, False)
    blue_serialized = blue_command.SerializeToString()
    yellow_serialized = yellow_command.SerializeToString()
    self.sock.sendto(blue_serialized, (COMMAND_GRP, COMMAND_PORT))
    self.sock.sendto(yellow_serialized, (COMMAND_GRP, COMMAND_PORT))
    #print("step")
    return self.get_state()
  def push_state(self):
    packet = grSim_Packet_pb2.grSim_Packet()
    reset = packet.replacement
    ball_reset = reset.ball
    ball_reset.x = self.ball.loc[0]/1000
    ball_reset.y = self.ball.loc[1]/1000
    ball_reset.vx = self.ball.velocity[0]
    ball_reset.vy = self.ball.velocity[1]
    for blue_bot in self.blue_robots:
      blue_init = reset.robots.add()
      blue_init.x = blue_bot.loc[0]/1000;
      blue_init.y = blue_bot.loc[1]/1000;
      blue_init.dir= blue_bot.rot;
      blue_init.id = blue_bot.id;
      blue_init.yellowteam = False;
    for yellow_bot in self.yellow_robots:
      yellow_init = reset.robots.add()
      yellow_init.x = yellow_bot.loc[0]/1000;
      yellow_init.y = yellow_bot.loc[1]/1000;
      yellow_init.dir= yellow_bot.rot;
      yellow_init.id = yellow_bot.id;
      yellow_init.yellowteam = True;
    #print("switch")
    self.sock.sendto(packet.SerializeToString(), (COMMAND_GRP, COMMAND_PORT))
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
 
  def make_command(self, actions, for_blue_team):
    #make command packet
    packet = grSim_Packet_pb2.grSim_Packet()
    packet.commands.timestamp = time.time()
    if for_blue_team:
      packet.commands.isteamyellow = False
    else:
      packet.commands.isteamyellow = True 
    i = 0
    for r in range(self.max_bots_per_team):
      #for each robot add a command
      comm = packet.commands.robot_commands.add()
      comm.id = i      
      comm.kickspeedx = actions[0+i*5]
      comm.kickspeedz = actions[1+i*5]
      comm.veltangent = actions[2+i*5]
      comm.velnormal = actions[3+i*5]
      comm.velangular = actions[4+i*5]
      
      #spinner always on
      comm.spinner = True
      #use velocity control
      comm.wheelsspeed = False
      i+=1
    return packet
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
  def update_from_proto(self, packet):
    frame = packet.detection
    #print(frame.frame_number, len(frame.balls), len(frame.robots_blue), len(frame.robots_yellow))
    #update the ball
    if (len(frame.balls) != 0):
      self.ball.update(np.array([frame.balls[0].x,frame.balls[0].y]), 1, time_stamp = frame.frame_number)
    else:
      self.ball.update(np.array([0, 0]), 0)

    blue_observed = [0 for _ in range(self.max_bots_per_team)]
    yellow_observed = [0 for _ in range(self.max_bots_per_team)]
    #for each instance observed update as observed
    for i in range(len(frame.robots_blue)):
      #print(i)
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

if __name__ == "__main__":
  max_bots_per_team = 8
  game = GRsim(max_bots_per_team)
  for j in range(10000):
    game.step([i%2 for i in range(5*max_bots_per_team)], [i%2 for i in range(5*max_bots_per_team)])
    #if j%200 == 0:
      #game.display()
    
