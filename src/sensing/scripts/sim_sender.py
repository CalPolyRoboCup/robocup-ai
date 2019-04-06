from socket import socket
from socket import AF_INET, SOCK_DGRAM, SOL_SOCKET, SO_REUSEADDR

from proto.grSim_Packet_pb2 import grSim_Packet


class SimSender:
    """
    Sends command packets to grSim
    """
    def __init__(self, address='127.0.0.1', port=20011):
        """
        Initializes a new SimSender instance
        :param address: The address to send the command packets to
        :param port: The port to send the command packets to
        """
        self.address = address
        self.port = port
        self.socket = None

    def connect(self):
        """
        Initializes the socket and connects to grSim
        """
        self.socket = socket(AF_INET, SOCK_DGRAM)
        self.socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
        self.socket.connect((self.address, self.port))

    def send(self, packet):
        """
        Sends the given packet to grSim
        :param packet: The grSim_Packet to send to grSim
        """
        self.socket.send(packet.SerializeToString())

