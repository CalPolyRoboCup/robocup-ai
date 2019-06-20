import sys
#replace this with your path to robocup-ai
sys.path.insert(0, '/home/nathan/Documents/robocup2018/robocup-ai/src')
from basic_skills.robot import *

#for vector math
import numpy as np

#for netowrorking
import socket
import struct

#current system time
import time
#to import proto

#protobuf implamentations expected in directory "proto" from run dirrectory
import GR_sim_networking.proto.grSim_Commands_pb2 as grSim_Commands_pb2
import GR_sim_networking.proto.grSim_Replacement_pb2 as grSim_Replacement_pb2
import GR_sim_networking.proto.grSim_Packet_pb2 as grSim_Packet_pb2
import GR_sim_networking.proto.messages_robocup_ssl_detection_pb2 as messages_robocup_ssl_detection_pb2
import GR_sim_networking.proto.messages_robocup_ssl_wrapper_pb2 as messages_robocup_ssl_wrapper_pb2
import GR_sim_networking.proto.referee_pb2 as referee_pb2
import matplotlib.pyplot as plt

'''
GRsimGame environment documentation

inputs:
kickspeedx 0
kickspeedz 1
veltangent 2
velnormal    3
velangular 4

output:
ball                     x, y, observed
blue_robots        x, y, orientation, observed
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
        self.observed = obs
        if (((time_stamp == None) or (time_stamp != self.last_timestamp)) and 
                obs and np.abs(nloc[0]) < 6000 and np.abs(nloc[1]) < 4000):        
            self.last_timestamp = time_stamp
            self.velocity = (self.velocity * self.smoothing + 
                (1-self.smoothing) * (nloc - self.loc)/time_elapsed)
            self.loc = ((self.loc + self.velocity*time_elapsed) * self.smoothing + 
                (1-self.smoothing) * (nloc))

def min_angle(angle):
    while 1:
        if angle > np.pi:
            angle -= 2*np.pi
        elif angle < -np.pi:
            angle += 2*np.pi
        else:
            return angle
        
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
        self.blue_robots = [robot(True, i, self) for i in range(self.max_bots_per_team)]
        self.yellow_robots = [robot(False, i, self
) for i in range(self.max_bots_per_team)]
        self.ball = ball()
    #use matplotlib to dispaly current game state
    #this function is slow you should not call it every frame    
    def display(self, keypoints = []):
        plt.figure(1)
        #plot important points
        plt.scatter([kp[0] for kp in keypoints], [kp[1] for kp in keypoints], color = "red", s = 250)

        #plot blue robot locations
        plt.scatter([br.loc[0] for br in self.blue_robots], [br.loc[1] for br in self.blue_robots], 
            color = 'b', s = 100)
        #plot yellow robot locations
        plt.scatter([yr.loc[0] for yr in self.yellow_robots], [yr.loc[1] for yr in self.yellow_robots],
            color = 'y', s = 100)

        for br in self.blue_robots:
            #plot velocity line
            plt.plot([br.loc[0], br.loc[0] + 0.2*br.velocity[0]], 
                             [br.loc[1], br.loc[1] + 0.2*br.velocity[1]], color = 'black', linewidth = 3)
            #plot rotation line    #TODO fix        
            plt.plot([br.loc[0] + 300*np.cos(br.rot)], 
                             [br.loc[1] + 300*np.sin(br.rot)], color = 'b', linewidth = 2)

        for yr in self.yellow_robots:
            #plot velocity line
            plt.plot([yr.loc[0], yr.loc[0] + 0.2*yr.velocity[0]], 
                             [yr.loc[1], yr.loc[1] + 0.2*yr.velocity[1]], color = 'black', linewidth = 3)
            #plot rotation line    #TODO fix 
            plt.plot([yr.loc[0] + 170*np.cos(yr.rot)], 
                             [yr.loc[1] + 170*np.sin(yr.rot)], color = 'yellow', linewidth = 2)
        #plot ball location
        plt.scatter([self.ball.loc[0]], [self.ball.loc[1]], color = 'g')
        #plot ball velocity line        
        plt.plot([self.ball.loc[0], self.ball.loc[0] + 0.2*self.ball.velocity[0]], 
                         [self.ball.loc[1], self.ball.loc[1] + 0.2*self.ball.velocity[1]], 
                         color = 'green', linewidth = 3)
        #show plot        
        plt.show(block = False)
        plt.pause(1E-12)
        plt.clf()
    #advance simulation first take
    #this function does not agree with current action architecture
    def step(self):
        self.sync_with_sim()
        for blue_robot in self.blue_robots:
            blue_command = self.make_command(blue_robot)
            if None == blue_command:
                continue
            blue_serialized = blue_command.SerializeToString()
            self.sock.sendto(blue_serialized, (COMMAND_GRP, COMMAND_PORT))
        for yellow_robot in self.yellow_robots:
            yellow_command = self.make_command(yellow_robot)
            if None == yellow_command:
                continue
            yellow_serialized = yellow_command.SerializeToString()
            self.sock.sendto(yellow_serialized, (COMMAND_GRP, COMMAND_PORT))
        return self.get_state()

    #sends a reset command to GRsim based on the current state
    #TODO? only update observed values
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
        self.sock.sendto(packet.SerializeToString(), (COMMAND_GRP, COMMAND_PORT))
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
        comm = packet.commands.robot_commands.add()
        comm.id = robot.id 
        comm.kickspeedx = actions[0]
        comm.kickspeedz = actions[1]
        comm.veltangent = actions[2]
        comm.velnormal = actions[3]
        comm.velangular = actions[4]
            
        #spinner always on
        comm.spinner = True
        #use velocity control
        comm.wheelsspeed = False
        return packet
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
        
