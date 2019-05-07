#!/usr/bin/env python
import xml.etree.ElementTree as ET
from grSim_Packet_pb2 import grSim_Packet
from grSim_Replacement_pb2 import grSim_Replacement, grSim_RobotReplacement
from socket import socket, AF_INET, SOCK_DGRAM, IPPROTO_UDP

class Ball:
    def __init__(self, x, y):
        self.x = x
        self.y = y

class Player:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
        
class GRSimConfig:
    def __init__(self, config_file):
        self.config_file = config_file
        self.ball = None
        self.players = []
        self.sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
        self.addr = '127.0.0.1'
        self.port = 20011

    def read_config(self):
        tree = ET.parse(self.config_file)
        root = tree.getroot()
        for child in root:
            if child.tag == 'ball':
                self.ball = Ball(0, 0)
                for ball_loc in child:
                    if ball_loc.tag == 'x':
                        self.ball.x = float(ball_loc.text)
                    if ball_loc.tag == 'y':
                        self.ball.y = float(ball_loc.text)
            if child.tag == 'player':
                tmp_player = Player(0, 0, 0)
                for player_pose in child:
                    if player_pose.tag == 'x':
                        tmp_player.x = float(player_pose.text)
                    if player_pose.tag == 'y':
                        tmp_player.y = float(player_pose.text)
                    if player_pose.tag == 'theta':
                        tmp_player.theta = float(player_pose.text)
                
                self.players.append(tmp_player)

    def sendto_grsim(self):
        pkt = grSim_Packet()
        num_robots = len(self.players)
        if self.ball is not None:
            print 'hi'
            pkt.replacement.ball.x = self.ball.x
            pkt.replacement.ball.y = self.ball.y
            pkt.replacement.ball.vx = 0.0
            pkt.replacement.ball.vy = 0.0
        
        pkt.replacement.robots.extend([grSim_RobotReplacement()] * num_robots)
        for i in range(num_robots):
            pkt.replacement.robots[i].x = self.players[i].x
            pkt.replacement.robots[i].y = self.players[i].y
            pkt.replacement.robots[i].dir = self.players[i].theta
            pkt.replacement.robots[i].id = i
            pkt.replacement.robots[i].yellowteam = False
            pkt.replacement.robots[i].turnon = False
        
        msg = pkt.SerializeToString()
        self.sock.sendto(msg, (self.addr, self.port))

    def setup_grsim(self):
        self.read_config()
        self.sendto_grsim()

if __name__ == '__main__':
    config_parser = GRSimConfig('../../config/sample.xml')
    config_parser.setup_grsim()
