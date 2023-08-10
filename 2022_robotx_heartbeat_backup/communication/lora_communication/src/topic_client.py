#!/usr/bin/env python

import socket
import rospy
from std_msgs.msg import Float32

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 12345        # Port to listen on (non-privileged ports are > 1023)


class Client(object):
    def __init__(self):

        self.pub_map = rospy.Publisher("/map_test",Float32,queue_size=1)

        self.s =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((HOST,PORT))
        data = self.s.recv(2048)
        rospy.loginfo("receive %s"%data)
        try:
            while True:
                data = self.s.recv(2048)
                rospy.loginfo("receive %s"%data)
                #self.pub_map(data)
        except KeyboardInterrupt:
            exit(0)

    def on_shutdown(self):
        self.s.close()
        

if __name__ == "__main__":
    rospy.init_node('map_topic_client')
    client = Client()
    rospy.on_shutdown(client.on_shutdown)
    rospy.spin()

