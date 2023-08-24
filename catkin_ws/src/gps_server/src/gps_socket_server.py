#!/usr/bin/env python3
import socket
import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import threading
import time
import datetime 
import json

class gpssub():
    def __init__(self, host, port):
        self.send_data = {
            "Time": datetime.datetime.strftime(datetime.datetime.now(), "%Y/%m/%d %H:%M:%S.%f")[:-3],
            "longitude": None,
            "latitude": None,
            "compass": None
        }

        self.gps_sub = rospy.Subscriber('/wamv_gps', NavSatFix, self.gps_callback)
        self.compass_sub = rospy.Subscriber('/wamv_compass', Float64, self.compass_callback)


        self.HOST = host
        self.PORT = port
        socket.setdefaulttimeout(86400*7)
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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
            DOS_flag = False
            for addr_ip in self.address.keys():
                if addr[0] == addr_ip[0]:
                    try:
                        reject = "more than 1 client link with this IP, connection reject"
                        client.send(reject.encode('utf-8'))
                        time.sleep(5)
                    except Exception as e:
                        print("Error sending reject message:", str(e))
                    finally:
                        client.close()
                    DOS_flag = True
                else:
                    pass

            #print('addr is ', addr)
            #print('client is ', client)
            if not DOS_flag:
                self.address[addr] = client
                print(datetime.datetime.strftime(datetime.datetime.now(), "%Y/%m/%d %H:%M:%S"), end = ' ')
                print('connection from ', addr)
                print(len(self.address), ' clients are connected')


    # def accept_client(self):
    #     while True:
    #         client, addr = self.server.accept()
    #         #print('addr is ', addr)
    #         #print('client is ', client)
    #         self.address[addr] = client
    #         print(datetime.datetime.strftime(datetime.datetime.now(), "%Y/%m/%d %H:%M:%S"), end = ' ')
    #         print('connection from ', addr)
    #         print(len(self.address), ' clients are connected')



    def broadcast(self):
        while True:
            self.send_data_string = json.dumps(self.send_data, indent=4)
            if self.send_data_string != '':
                disconnect = []
                for addr in self.address.keys():
                    try:
                        self.address[addr].send(self.send_data_string.encode('utf-8'))
                    except:
                        disconnect.append(addr)

                for addr in disconnect:
                    print(datetime.datetime.strftime(datetime.datetime.now(), "%Y/%m/%d %H:%M:%S"), end = ' ')
                    print(addr, ' is disconnected')
                    del(self.address[addr])
                    print(len(self.address), ' clients are connected')
                
                
                
                    #print('send to ', client)
                    #print('gps data is ', self.gps_data)
                    #print('send successfully')
            time.sleep(0.1)
            self.send_data['Time'] = datetime.datetime.strftime(datetime.datetime.now(), "%Y/%m/%d %H:%M:%S.%f")[:-3]
            
        

    def gps_callback(self, msg):
        self.send_data['latitude'] = msg.latitude
        self.send_data['longitude'] = msg.longitude
        # self.send_data['altitude'] = msg.altitude
        
    def compass_callback(self, msg):
        self.send_data['compass'] = msg.data

if __name__ == '__main__':
    rospy.init_node('gps_server')
    #host = input("Enter the IP address of the server: ")
    host="140.113.148.110"
    gps_sub = gpssub(host, 8888)
    
    rospy.spin()
