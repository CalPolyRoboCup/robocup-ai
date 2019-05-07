#!/usr/bin/env python
import socket
import struct
import select
from messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket

'''
Class for UDP multicast socket for listening to SSL Vision
'''
class SSLReceiver():
    def __init__(self, addr='224.5.23.2', port=10020):
        self.addr = addr
        self.port = port
        self.sock = None
        self.rd = None

    '''
    Open socket and connect to multicast group
    '''
    def open(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.addr, self.port))
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, struct.pack('=4sl', socket.inet_aton(self.addr), socket.INADDR_ANY))
        self.rd = [self.sock]
    
    '''
    Receive data from SSL Vision
    '''
    def recv_msg(self):
        buff = select.select(self.rd, [], [], 0)
        for fd in buff[0]:
            if fd is self.sock:
                data = self.sock.recv(2048)
                try:
                    return SSL_WrapperPacket().FromString(data)
                except DecodeError:
                    print("Decode Error")
                    return None
        return None       
