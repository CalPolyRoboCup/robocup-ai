import socket

IP = '224.5.23.3'
PORT = 10021

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sock.bind((IP, PORT))
mreq = struct.pack("4s1", socket.inet_aton(IP), socket.INADDR_ANY)
sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

while(1):
    data = sock.recv(1024)
    m1 = data[:2]
    m2 = data[2:4]
    m3 = data[4:6]
    m4 = data[6:8]
    kick = data[8]
    spinner = data[9:11]

    print(m1, m2, m3, m4, kick, spinner)
