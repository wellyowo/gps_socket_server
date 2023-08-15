#!/usr/bin/env python3
import socket
import rospy
from sensor_msgs.msg import NavSatFix
import threading
import time
import datetime 
import json

class gpssub():
    def __init__(self, host, port):
        self.gps_sub = rospy.Subscriber('/wamv/sensors/gps/gps/fix', NavSatFix, self.gps_callback)
        self.HOST = host
        self.PORT = port
        socket.setdefaulttimeout(86400)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.bind((self.HOST, self.PORT))
        self.server.listen(5)
        self.address = {}
        print('server start at ', self.server.getsockname())
        print('wait for connection...')
        self.gps_data = ''
        accept_thread = threading.Thread(target=self.accept_client, daemon=True)
        broadcast_thread = threading.Thread(target=self.broadcast, daemon=True)
        accept_thread.start()
        broadcast_thread.start()
        

    def accept_client(self):
        while True:
            client, addr = self.server.accept()
            print('addr is ', addr)
            print('client is ', client)
            self.address[addr] = client
            print('connection from ', addr)
        

    def broadcast(self):
        while True:
            if self.gps_data != '':
                disconnect = []
                for addr in self.address.keys():
                    try:
                        self.address[addr].send(self.gps_data.encode('utf-8'))
                    except:
                        print(addr, ' is disconnected')
                        disconnect.append(addr)
                for addr in disconnect:
                    del(self.address[addr])
                
                    #print('send to ', client)
                    #print('gps data is ', self.gps_data)
                    #print('send successfully')
            time.sleep(1)
        

    def gps_callback(self, msg):
        data = {
            "Time": datetime.datetime.now().strftime("%Y/%m/%d %H:%M:%S"),
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude
        }
        self.gps_data = json.dumps(data, indent=4)
        #print(self.gps_data)

if __name__ == '__main__':
    rospy.init_node('gps_server')
    #host = input("Enter the IP address of the server: ")
    host="140.113.148.110"
    gps_sub = gpssub(host, 8888)
    
    rospy.spin()
