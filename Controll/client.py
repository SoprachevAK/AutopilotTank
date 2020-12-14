import socket
import time
from config import SOCKET_PARAMS, sockets


if __name__ == '__main__':
    s = socket.socket(*SOCKET_PARAMS)
    s.connect(sockets["inSoc"])

    try:
        while True:
            s.sendall(input().encode())
            data = s.recv(1024)
            print('Received', repr(data))
            # time.sleep(1)
    finally:
        s.close()