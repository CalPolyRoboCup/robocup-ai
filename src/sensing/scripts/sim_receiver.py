#!/usr/bin/env python

from socket import socket, inet_aton, timeout
from socket import AF_INET, SOCK_DGRAM, SOL_SOCKET, SO_REUSEADDR, IPPROTO_IP, IP_ADD_MEMBERSHIP, INADDR_ANY

from ipaddress import ip_address

from struct import pack

from google.protobuf.message import DecodeError

from messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket

import logging


class SimReceiver:
    """
    Used to receive vision packets from grSim
    """
    TIMEOUT = 0.5
    PACKET_SIZE = 2048

    def __init__(self, address='224.5.23.2', port=10020):
        """
        Initializes the SimConnection instance
        :param address: The grSim vision address
        :param port: The grSim vision port
        """
        logging.basicConfig()

        self.address = address
        self.port = port
        self.logger = logging.getLogger(__name__)
        self.socket = None

    def open(self):
        """
        Opens a connection to grSim with the previously specified information
        """
        self.socket = socket(AF_INET, SOCK_DGRAM)
        self.socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        self.socket.bind((self.address, self.port))

        # Use slightly different network settings if grSim is using a multicast protocol
        if ip_address(self.address).is_multicast:
            self.socket.setsockopt(IPPROTO_IP, IP_ADD_MEMBERSHIP,
                                   pack('=4sl', inet_aton(self.address), INADDR_ANY))

        self.socket.settimeout(SimReceiver.TIMEOUT)

    def receive(self):
        """
        Reads an SSL_WrapperPacket from grSim and returns the result
        :raise: RuntimeError if the connection has not been opened
        :return: The SSL_WrapperPacket decoded from grSim, or None if an error occurred
        """
        if self.socket is None:
            raise RuntimeError('Cannot receive packets before a connection is opened')

        try:
            data = self.socket.recv(SimReceiver.PACKET_SIZE)
        except timeout:
            self.logger.log(logging.ERROR, 'Socket timed out when receiving a packet')
            return None

        try:
            return SSL_WrapperPacket().FromString(data)
        except DecodeError:
            self.logger.error(logging.ERROR, 'Could not decode a SSL_WrapperPacket')
            return None
