import socket
import threading
import sys


def receive():
    while True:
        try:
            msg = client_socket.recv(BUFSIZE).decode()
            print(msg)
        except OSError:
            break




if __name__ == '__main__':
    # HOST = input('Enter host IP: ')
    # PORT = input('Enter port: ')
    HOST  = '140.113.148.110'
    PORT = 8888
    PORT = int(PORT)
    BUFSIZE = 1024

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((HOST, PORT))
    #threading.Thread(target=receive).start()
    while True:
        try:
            msg = client_socket.recv(BUFSIZE).decode()
            print(msg)
        except OSError:
            break