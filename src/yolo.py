import socket

host = '0.0.0.0'

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
    s.bind(('0.0.0.0', 11301))
    # s.connect((host,port))
    s.sendto(b"Hello from plankton", ('127.0.0.1', 11300))
    #s.send(b"Hello from plankton")
    buf = s.recv(2048)
    print(buf)
