#!/usr/bin/env python

import rospy
import sys
import socket
import struct
import fcntl, os
import errno
import numpy
from time import sleep
from messages_robocup_ssl_wrapper_pb2 import *
from google.protobuf.message import DecodeError
from data_processing.msg import PlayerData
from data_processing.msg import BallData
from data_processing.msg import FieldData

#socket addresses
MULTICAST = '224.5.23.2'
server_addr = ('', 10020)

#time constant (60 fps)
DELTA_TIME = 1.0/60.0
def update_world():
    #create publisher
    pub = rospy.Publisher('field_data', FieldData, queue_size = 10)
    rospy.init_node('ssl_vision', anonymous=True)
    rate = rospy.Rate(60)
    #initialize protobuf packet
    wrapper_packet = SSL_WrapperPacket()
    last_frame = 0
    delta_time = DELTA_TIME
    #initialize message
    msg = FieldData()
    msg.ball = BallData()
    last_ball_x = 0
    last_ball_y = 0
    msg.opp_players = numpy.empty(6, PlayerData)
    opp_last = numpy.empty(6, PlayerData)
    for i in xrange(6):
        opp_last[i] = PlayerData()
    msg.team_players = numpy.empty(6,PlayerData)
    team_last = numpy.empty(6, PlayerData)
    for i in xrange(6):
         team_last[i] = PlayerData()
    
    while not rospy.is_shutdown():
        try:
	    #receive data from socket
            data, addr = sock.recvfrom(1024)
        except socket.error, e:
            err = e.args[0]
            if err == errno.EAGAIN or err == errno.EWOULDBLOCK:
                sleep(1)
                print "No data available"
                continue
            else:
                print e
                sys.exit(1)
        else:
            #print data
            try:
                wrapper_packet.ParseFromString(data)
            except DecodeError, e:
                print e
                continue
            else: 
                #print addr
                f_data = wrapper_packet.detection
                #print last_frame
                #print f_data.frame_number
                # update time between frame
                if not last_frame == f_data.frame_number:
                    delta_time = (last_frame - f_data.frame_number) * delta_time
                    last_frame = f_data.frame_number
                
                #update ball position
                if len(f_data.balls) > 0:
                    msg.ball.x = f_data.balls[0].x
                    msg.ball.y = f_data.balls[0].y
                    msg.ball.v_x = (last_ball_x - f_data.balls[0].x) / delta_time
                    msg.ball.v_y = (last_ball_y - f_data.balls[0].y) / delta_time
                #update our bots positions
                if len(f_data.robots_yellow) > 0:
                    for robot in f_data.robots_yellow:
                        r_id = robot.robot_id
                        msg.team_players[r_id] = PlayerData()
                        msg.team_players[r_id].x = robot.x
                        msg.team_players[r_id].y = robot.y
                        msg.team_players[r_id].v_x = (team_last[r_id].x - robot.x) / delta_time
                        msg.team_players[r_id].v_y = (team_last[r_id].y - robot.y) / delta_time
                        msg.team_players[r_id].angle = robot.orientation
                        msg.team_players[r_id].angular_velocity = (team_last[r_id].angle - robot.orientation) / delta_time
                        team_last[r_id] = msg.team_players[r_id] 
                #update opp bots positions
                if len(f_data.robots_blue) > 0:
                    for robot in f_data.robots_blue:
                        r_id = robot.robot_id
                        msg.opp_players[r_id] = PlayerData()
                        msg.opp_players[r_id].x = robot.x
                        msg.opp_players[r_id].y = robot.y
                        msg.opp_players[r_id].v_x = (opp_last[r_id].x - robot.x) / delta_time
                        msg.opp_players[r_id].v_y = (opp_last[r_id].y - robot.y) / delta_time
                        msg.opp_players[r_id].angle = robot.orientation
                        msg.opp_players[r_id].angular_velocity = (opp_last[r_id].angle - robot.orientation) / delta_time
                        opp_last[r_id] = msg.opp_players[r_id]
                #publish the message
                print msg.ball is None
                print msg.opp_players is None
                print msg.team_players is None
                pub.publish(msg) 
if __name__ == '__main__':
    try:
        #setup socket to listen
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(server_addr)
        group = socket.inet_aton(MULTICAST)
        mreq = struct.pack('4sL', group, socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,mreq)
        fcntl.fcntl(sock, fcntl.F_SETFL, os.O_NONBLOCK)
        update_world()
    except socket.error, e:
        print e
        sys.exit(1)
