#!/usr/bin/env python

import socket
import rospy
from std_msgs.msg import Float32

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 12345        # Port to listen on (non-privileged ports are > 1023)


class Server(object):
    def __init__(self):

        self.sub_map = rospy.Subscriber("/map_test",Float32,self.cb_map,queue_size=1)

        self.s =  socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((HOST, PORT))
        self.s.listen(0)
        self.conn, self.addr = self.s.accept()
        rospy.loginfo('connectoin:{},{}'.format(self.conn,self.addr))
    
    def cb_map(self,msg):
        with self.conn:
            self.conn.sendall(msg.data)
    
    def on_shutdown(self):
        self.conn.close()
        self.s.close()
        

if __name__ == "__main__":
    rospy.init_node('map_topic_server')
    server = Server()
    rospy.on_shutdown(server.on_shutdown)
    rospy.spin()

