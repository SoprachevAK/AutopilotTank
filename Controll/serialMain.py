import serial
import time
import select
import socket
from config import SOCKET_PARAMS, sockets


def send(s):
    print(s + b";")
    ser.write(s + b";")


ser = serial.Serial("/dev/tty.usbserial-1430", 115200)


def init_serial():
    start = False

    while not start:
        while ser.in_waiting:
            res = ser.readline().decode()
            print(res)
            if res == "DONE\r\n":
                start = True
        time.sleep(0.1)


if __name__ == '__main__':
    init_serial()

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind(sockets["inSoc"])

    while True:
        sock.listen()
        print("accept")
        conn, a = sock.accept()
        print(a)

        try:
            while True:
                reader = select.select([conn], [], [], 0.0)
                if reader[0]:
                    data = conn.recv(1024)
                    if data != b'':
                        send(data)

                while ser.in_waiting:
                    res = ser.readline()
                    conn.send(res)
        except BrokenPipeError as e:
            pass
        except ConnectionResetError as e:
            pass
