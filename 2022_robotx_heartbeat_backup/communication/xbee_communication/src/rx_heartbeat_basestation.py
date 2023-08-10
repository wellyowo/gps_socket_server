#!/usr/bin/python3           
# This is client.py file


# import socket

# HOST = "127.0.0.1"  # Standard loopback interface address (localhost)
# PORT = 65432  # Port to listen on (non-privileged ports are > 1023)

import socket
import rospy
from std_msgs.msg import Bool , Int32,String,Float32
from sensor_msgs.msg import NavSatFix
import datetime


class heartbeat_program_base(object):
    def __init__(self):
    # get the hostname
        # self.server  = socket.gethostbyname("robot.server")
        # print(self.server)
        self.host = "robot.server"
        self.port = 12345  # initiate port no above 1024

        self.client_socket = socket.socket()  # get instance
        self.client_socket.connect((self.host, self.port))
        self.sub_sentence = rospy.Subscriber("/heartbeat/sentence", String, self.cb_sentence, queue_size=1)
        self.sub_sentence_xbee = rospy.Subscriber("/heartbeat/sentence/xbee", String, self.cb_sentence_xbee, queue_size=1)
        self.count = 5
        self.timer = rospy.Timer(rospy.Duration(1), self.cb_heartbeat)

        self.data = ""

    def cb_heartbeat(self,event):
        # receive data stream. it won't accept data packet greater than 1024 bytes
        #message = "fuck"
        #print("fuck")
        print(self.data)
        self.count = self.count - 1
        self.client_socket.send(self.data.encode( )) 
        #self.conn.send(self.data.encode())  # send data to the client

    def cb_sentence(self,msg):
        self.data = msg.data
        self.count = 5

    def cb_sentence_xbee(self,msg):
        if (self.count < 0):
            self.data = msg.data


if __name__ == '__main__':
    rospy.init_node("heartbeat_program_base")
    Heartbeat_program_base = heartbeat_program_base()
    rospy.spin()