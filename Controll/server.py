print("server")

import socket
from config import sockets as s

SOCKET_PARAMS = socket.AF_INET, socket.SOCK_STREAM

inSoc = socket.socket(*SOCKET_PARAMS)
outSoc = socket.socket(*SOCKET_PARAMS)

inSoc.bind(s["inSoc"])
outSoc.bind(s["outSoc"])

while True:
    inSoc.listen()
    print("wait connection")
    conn, addr = inSoc.accept()
    s.settimeout(0)

    while True:
        try:
            data = conn.recv(1024)

            if not data:
                break

            conn.sendall(data)
        except socket.timeout as e:
            print(e, "again")