#!/usr/bin/env python
import xml.etree.ElementTree as ET
from grSim_Packet_pb2 import grSim_Packet
from grSim_Replacement_pb2 import grSim_Replacement, grSim_RobotReplacement
from socket import socket, AF_INET, SOCK_DGRAM, IPPROTO_UDP
from robocup_master.config_data import Config, Robot, Ball
from robocup_master.config_handler import *

class GRSimConfig:
    def __init__(self, config_file):
        self.config_file = config_file
        self.config = Config()
        self.sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)
        self.addr = '127.0.0.1'
        self.port = 20011

    def sendto_grsim(self):
        pkt = grSim_Packet()
        num_yellow = len(self.config.robots_yellow)
        num_blue = len(self.config.robots_blue)
        if self.config.ball is not None:
            print 'hi'
            pkt.replacement.ball.x = self.config.ball.x / 100.0
            pkt.replacement.ball.y = self.config.ball.y / 100.0
            pkt.replacement.ball.vx = 0.0
            pkt.replacement.ball.vy = 0.0
        
        pkt.replacement.robots.extend([grSim_RobotReplacement()] * (num_yellow + num_blue))
        for i in range(num_yellow):
            pkt.replacement.robots[i].x = self.config.robots_yellow[i].pose.x / 100.0
            pkt.replacement.robots[i].y = self.config.robots_yellow[i].pose.y / 100.0
            pkt.replacement.robots[i].dir = self.config.robots_yellow[i].pose.theta / 100.0
            pkt.replacement.robots[i].id = i
            pkt.replacement.robots[i].yellowteam = True
            pkt.replacement.robots[i].turnon = False
        offset = num_yellow;
        for i in range(num_blue):
            pkt.replacement.robots[i+offset].x = self.config.robots_blue[i].pose.x / 100.0
            pkt.replacement.robots[i+offset].y = self.config.robots_blue[i].pose.y / 100.0
            pkt.replacement.robots[i+offset].dir = self.config.robots_blue[i].pose.theta / 100.0
            pkt.replacement.robots[i+offset].id = i
            pkt.replacement.robots[i+offset].yellowteam = False
            pkt.replacement.robots[i+offset].turnon = False

        msg = pkt.SerializeToString()
        self.sock.sendto(msg, (self.addr, self.port))

    def setup_grsim(self):
        self.config = read_placement(self.config_file)
        self.sendto_grsim()

if __name__ == '__main__':
    config_parser = GRSimConfig('../../config/sample.place')
    config_parser.setup_grsim()
