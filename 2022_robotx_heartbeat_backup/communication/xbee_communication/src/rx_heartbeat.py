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


class server_program(object):
    def __init__(self):
    # get the hostname
        self.server  = socket.gethostbyname("robot.server")
        print(self.server)
        self.host = "127.0.0.1"
        self.port = 12345  # initiate port no above 1024

        self.client_socket = socket.socket()  # get instance
        # look closely. The bind() function takes tuple as argument
        #self.server_socket.bind((self.host, self.port))  # bind host address and port together
        self.client_socket.connect((self.server, self.port))
        # configure how many client the server can listen simultaneously
        #self.client_socket.listen(2)
        #self.conn, self.address = self.client_socket.accept()  # accept new connection
        #print("Connection from: " + str(self.address))
        self.sub_gps = rospy.Subscriber("/wamv/mavros/global_position/global", NavSatFix, self.cb_gps, queue_size=1)
        #self.sub_gps = rospy.Subscriber("/wamv/sensors/gps/gps/fix", NavSatFix, self.cb_gps, queue_size=1)
        self.sub_auto = rospy.Subscriber("/auto_state", Bool, self.cb_auto, queue_size=1)
        self.sub_estop = rospy.Subscriber("/stop_state", Bool, self.cb_estop, queue_size=1)
        self.sub_exit = rospy.Subscriber("/wamv/gate_num",Float32, self.cb_exit, queue_size=1)
        self.sub_drone_state = rospy.Subscriber("/drone_heartbeat", Int32, self.cb_uav, queue_size=1)
        self.sub_scan = rospy.Subscriber("/wamv/find_seq", String, self.cb_scan, queue_size=1)
        self.sub_dock_color = rospy.Subscriber("/wamv/dock_color", String, self.cb_dock_color, queue_size=1)
        self.sub_fling_color = rospy.Subscriber("/wamv/fling_color", String, self.cb_fling_color, queue_size=1)
        self.sub_dock_status = rospy.Subscriber("/docking_finished_success", Bool, self.cb_dock_status, queue_size=1)
        self.sub_fling_status = rospy.Subscriber("/nav_to_fling_finished_success", Bool, self.cb_fling_status, queue_size=1)
        self.sub_drone_search = rospy.Subscriber("/search_heartbeat", String, self.cb_uav_search, queue_size=1) # "R,24.5,S,122.7,E"
        self.sub_drone_rep = rospy.Subscriber("/replenishment_heartbeat", Int32, self.cb_uav_rep, queue_size=1)
        self.sub_follow_path = rospy.Subscriber("/follow_path_heartbeat", Int32, self.cb_follow, queue_size=1)
        self.now = datetime.datetime.now()
        self.year = str(self.now.year)[-2:]
        self.month = str(self.now.month)
        self.day = str(self.now.day)
        self.hour = str(self.now.hour)
        self.minute = str(self.now.minute)
        self.second = str(self.now.second)
        self.latitude = 33.72037
        self.longitude = 150.67115
        self.uav_search_status = ""

        self.auto = 0
        self.entrance = 1
        self.exit = 1
        #
        self.follow_path = 1
        self.num_detect = 3
        #
        self.scan_code = "RBG"
        self.dock_color = "R"
        self.dock_status = 1
        self.fling_color = "R"
        self.fling_status = 1
        self.rep_status = 0

        self.timer = rospy.Timer(rospy.Duration(1), self.cb_heartbeat)
        self.timer_1 = rospy.Timer(rospy.Duration(1), self.cb_gettime)
        self.timer_2 = rospy.Timer(rospy.Duration(1), self.cb_filldata)

        self.data = ""
        self.mode = 1
        self.uav_mode = 1
        self.teamid = "NYCU"
        #print("fuckfuck")
        self.count = 0

    def cb_heartbeat(self,event):
        # receive data stream. it won't accept data packet greater than 1024 bytes
        #message = "fuck"
        #print("fuck")
        print(self.data)
        self.client_socket.send(self.data.encode( ))
        self.count = self.count + 1
        if (self.count > 5):
            self.mode = 2
            self.uav_mode = 2 
        #self.conn.send(self.data.encode())  # send data to the client

    def cb_gettime(self,event):
        self.now = datetime.datetime.now()
        self.year = str(self.now.year)[-2:]
        self.month = str(self.now.month)
        self.day = str(self.now.day)
        self.hour = str(self.now.hour)
        if self.now.hour < 10:
            self.hour = "0"+self.hour
        self.minute = str(self.now.minute)
        if self.now.minute < 10:
            self.minute = "0"+self.minute
        self.second = str(self.now.second)
        if self.now.second < 10:
            self.second = "0"+self.second

    def cb_filldata(self,event):
        self.heartbeat_task()
        if (self.count>10):
            self.entrance_task()



    def cb_exit(self,msg):
        self.entrance = int(msg.data)
        self.exit = int(msg.data)

    def cb_scan(self,msg):
        self.scan_code = msg.data
        #print(self.scan_code)
        self.scan_code = self.scan_code[0]+self.scan_code[2]+self.scan_code[4]

    def cb_dock_status(self,msg):
        if msg.data :
            self.dock_status = 2
        else:
            self.dock_status = 1


    def cb_fling_status(self,msg):
        if msg.data :
            self.fling_status = 2
        else:
            self.fling_status = 1


    def cb_gps(self,msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def cb_auto(self,msg):
        self.auto = msg.data

    def cb_estop(self,msg):
        self.estop = msg.data
        if self.estop:
            self.mode = 3
        else:
            if self.auto:
                self.mode = 2
            else:
                self.mode = 1

    def cb_uav(self,msg):
        self.uav_mode = msg.data

    def cb_follow(self,msg):
        self.follow_path = msg.data

    def cb_uav_search(self,msg):
        self.uav_search_status = msg.data

    def cb_uav_rep(self,msg):
        self.rep_status = msg.data

    def cb_dock_color(self,msg):
        self.dock_color = msg.data

    def cb_fling_color(self,msg):
        self.fling_color = msg.data


    def checksum(self,sentence):
        len1 = len(sentence)

        ans = ord(sentence[0])
    
        for i in range(1,len1):
    
            # Traverse string to find the XOR
            ans = (ans ^ (ord(sentence[i])))
    
        # Return the XOR
        
        #print(str(hex(ans))[-2:])
        crc = (str(hex(ans))[-2:])
        crc = crc.upper()

        return crc

    def heartbeat_task(self):
        sentence = "RXHRB," +self.day+self.month+self.year+","+self.hour +self.minute+self.second+","+str(self.latitude)+",S,"+str(self.longitude)+ \
            ",E,"+ self.teamid + "," +str(self.mode)+","+str(self.uav_mode)
        check = self.checksum(sentence)

        self.data = "$"+sentence+"*"+check+"\r" + "\n"
        #print(self.data)

    # def dock_task(self):
    #     sentence = "RXDOK," +self.day+self.month+self.year+","+self.hour +self.minute+self.second+","+str(self.latitude)+",S,"+str(self.longitude)+ \
    #         ",E,"+ self.teamid + "," + 

    
    # def search_task(self):
    #     sentence = "RXSAR," +self.day+self.month+self.year+","+self.hour +self.minute+self.second+","+self.uav_search_status+","+ self.teamid + ","+str(self.uav_mode)
    #     check = self.checksum(sentence)

    #     self.data = "$"+sentence+"*"+check+"\r" + "\n"

    def entrance_task(self):
        sentence = "RXGAT," +self.day+self.month+self.year+","+self.hour +self.minute+self.second+","+ self.teamid+","+str(self.entrance)+ ","+str(self.entrance)
        check = self.checksum(sentence)

        self.data = "$"+sentence+"*"+check+"\r" + "\n"

    def followpath_task(self):
        sentence = "RXPTH," +self.day+self.month+self.year+","+self.hour +self.minute+self.second+","+ self.teamid+","+str(self.follow_path)
        check = self.checksum(sentence)

        self.data = "$"+sentence+"*"+check+"\r" + "\n"   

    def wildlife_task(self):
        sentence = "RXENC," +self.day+self.month+self.year+","+self.hour +self.minute+self.second+","+ self.teamid+","+str(self.follow_path)
        check = self.checksum(sentence)

        self.data = "$"+sentence+"*"+check+"\r" + "\n" 

    def scan_task(self):
        sentence = "RXCOD," +self.day+self.month+self.year+","+self.hour +self.minute+self.second+","+ self.teamid+ ","+str(self.scan_code)
        check = self.checksum(sentence)

        self.data = "$"+sentence+"*"+check+"\r" + "\n"   


    def replenishment_task(self):
        sentence = "RXUAV," +self.day+self.month+self.year+","+self.hour +self.minute+self.second+","+ self.teamid+ ","+str(self.uav_mode)+","+str(self.rep_status)
        check = self.checksum(sentence)

        self.data = "$"+sentence+"*"+check+"\r" + "\n"   

    def dock_task(self):
        sentence = "RXDOK," +self.day+self.month+self.year+","+self.hour +self.minute+self.second+","+ self.teamid+ ","+self.dock_color+","+str(self.dock_status)
        check = self.checksum(sentence)

        self.data = "$"+sentence+"*"+check+"\r" + "\n"   

    def fling_task(self):
        sentence = "RXFLG," +self.day+self.month+self.year+","+self.hour +self.minute+self.second+","+ self.teamid+ ","+self.fling_color+","+str(self.fling_status)
        check = self.checksum(sentence)

        self.data = "$"+sentence+"*"+check+"\r" + "\n"   






if __name__ == '__main__':
    rospy.init_node("Server_program")
    Server_program = server_program()
    rospy.spin()