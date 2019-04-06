import socket
import binascii
import messages_robocup_ssl_wrapper_pb2
import select
import struct

class SSLClient:
    
    def __init__(self, ip = '224.5.23.2', port=10020): #224.5.23.2
        """
        Init SSLClient object.

        Extended description of function.

        Parameters
        ----------
        ip : str
            Multicast IP in format '255.255.255.255'. 
        port : int
            Port up to 1024. 
        """
        
        self.ip = ip
        self.port = port

    def connect(self):
        """Binds the client with ip and port and configure to UDP multicast."""

        if not isinstance(self.ip, str):
            raise ValueError('IP type should be string type')
        if not isinstance(self.port, int):
            raise ValueError('Port type should be int type')
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        #self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
        #self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self.sock.bind((self.ip, self.port))
        '''
        host = socket.gethostbyname(socket.gethostname())
        self.sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, socket.inet_aton(self.ip))
        self.sock.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, 
                socket.inet_aton(self.ip) + socket.inet_aton(self.ip))
        '''
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, struct.pack('=4sl', socket.inet_aton(self.ip), socket.INADDR_ANY))
    def receive(self):
        """Receive package and decode."""
        sock_list = [self.sock]
        # list to check for reading, list to check for writing, list to check for error
        # sel_data is file descriptor to where we read from
        # 1st _ file desc to where we write to
        # 2nd _ file desc to error 
        sel_data, _, _ = select.select(sock_list, [],[], 0)
        if(sel_data==self.sock):
            data, _ = self.sock.recvfrom(1024)
            decoded_data = messages_robocup_ssl_wrapper_pb2.SSL_WrapperPacket().FromString(data)
            return decoded_data
        else: return "nothing"