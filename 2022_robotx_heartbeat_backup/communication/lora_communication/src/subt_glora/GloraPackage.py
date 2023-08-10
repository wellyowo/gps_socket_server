#!/usr/bin/env python
import serial
import struct
import math
import sys
import copy
import genpy

sys.path.insert(0, '/opt/ros/$ROS_DISTRO/lib/python2.7/dist-packages')
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String,Header, Float32MultiArray
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class GloraPackage(object):
    """docstring for GloraPackage"""
    def __init__(self, identity_byte=None, msg=None, time_stamp=None, position=None, rpy=None, data=0):
        self.pack = []

        #
        # PoseStamped message
        #
        if isinstance(msg, PoseStamped) and identity_byte != None:
            q_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(q_list)

            # 1B --> 1 char              1B
            # 8f --> 8 float(32bits) 4*8=32B
            self.pack = bytearray(struct.pack("=1B8f", 
                                        identity_byte,
                                        msg.header.stamp.to_sec(),
                                        msg.pose.position.x,
                                        msg.pose.position.y,
                                        msg.pose.position.z,
                                        roll,
                                        pitch,
                                        yaw,
                                        data))
            #self.generate_checksum_()

        #
        # Float32MultiArray 
        #
        elif isinstance(msg, Float32MultiArray) and identity_byte != None:
            # 1B --> 1 char              1B
            # 8f --> 8 float(32bits) 4*8=32B
            self.pack = bytearray(struct.pack("=1B16f", 
                                        identity_byte,
                                        msg.data[0],
                                        msg.data[1],
                                        msg.data[2],
                                        msg.data[3],
                                        msg.data[4],
                                        msg.data[5],
                                        msg.data[6],
                                        msg.data[7],
                                        msg.data[8],
                                        msg.data[9],
                                        msg.data[10],
                                        msg.data[11],
                                        msg.data[12],
                                        msg.data[13],
                                        msg.data[14],
                                        msg.data[15]))

        #
        # Odometry message
        #
        elif isinstance(msg, Odometry) and identity_byte != None:
            q_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(q_list)

            # 1B --> 1 char              1B
            # 8f --> 8 float(32bits) 4*8=32B
            self.pack = bytearray(struct.pack("=1B8f", 
                                        identity_byte,
                                        msg.header.stamp.to_sec(),
                                        msg.pose.pose.position.x,
                                        msg.pose.pose.position.y,
                                        msg.pose.pose.position.z,
                                        roll,
                                        pitch,
                                        yaw,
                                        data))
            #self.generate_checksum_()

        #
        # Pose message
        #
        elif isinstance(msg, Pose) and time_stamp != None and identity_byte != None:
            q_list = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
            (roll, pitch, yaw) = euler_from_quaternion(q_list)

            # 1B --> 1 char              1B
            # 8f --> 8 float(32bits) 4*8=32B
            self.pack = bytearray(struct.pack("=1B8f", 
                                        identity_byte,
                                        time_stamp.to_sec(),
                                        msg.position.x,
                                        msg.position.y,
                                        msg.position.z,
                                        roll,
                                        pitch,
                                        yaw,
                                        data))
            #self.generate_checksum_()

        #
        # When input include position and rpy parameters
        #
        elif isinstance(msg, type(None)) and time_stamp!=None and position!=None and rpy!=None and identity_byte != None:
            if(len(position)!=3 or len(rpy)!=3):
                raise ValueError('Parameters \'position\' and \'rpy\' need to be a list with length=3')
            
            if isinstance(time_stamp, genpy.rostime.Time):
                time_stamp = time_stamp.to_sec()

            self.pack = bytearray(struct.pack("=1B8f", 
                                        identity_byte,
                                        time_stamp,
                                        position[0],
                                        position[1],
                                        position[2],
                                        rpy[0],
                                        rpy[1],
                                        rpy[2],
                                        data))
            #self.generate_checksum_()

        elif isinstance(msg, type(None)) and identity_byte==None:
            pass

        else:
            raise KeyError('Message for GloraPackage is not existed.')
    

    def __repr__(self):
        return self.pack

    def __str__(self):
        return self.pack.__str__()

    def __len__(self):
        return len(self.pack)

    def generate_checksum_(self):
        checksum = 0x10000 - (sum(self.pack) % 0x10000)
        self.pack.extend((checksum // 256, checksum % 256))
        # print('checksum= {}'.format(checksum))

    def check_package(self, pack):
        if len(pack) < 2: return False 
        else:
            tmp_pack = copy.deepcopy(pack)
            num_l = tmp_pack.pop(-1)
            num_h = tmp_pack.pop(-1)
            checksum = num_h<<8 | num_l
            return (sum(tmp_pack) + checksum) & 0xffff == 0

    def extract_point(self, pack):
        try:
            data = struct.unpack("=" + "1B16f", pack)
            msg = Float32MultiArray()
            for k in range(16):
                msg.data.append(data[k+1])
            return (data[0], msg)

        except:
            print("data currupt")
            return (None, None)
        

    def unpack(self, pack, topic):
        try:
            num_of_info = (len(pack)-2) // 33
            idbytes_list = []
            msgs_list = []
            data_list = []

            struct.unpack
            data = struct.unpack("=" + "1B8f"*num_of_info + "2B", pack)
            for i in range(num_of_info):
                # Collect idbyte
                pack_len = 9
                idbytes_list.append(data[i*pack_len])

                #collect data
                data_list.append(data[8+i*pack_len]) 

                # Collect idby
                if isinstance(topic, PoseStamped):
                    q_list = quaternion_from_euler(data[5 + i*pack_len], data[6 + i*pack_len], data[7 + i*pack_len])
                    msg = PoseStamped()
                    msg.header.stamp = rospy.Time.from_sec(data[1 + i*pack_len])
                    msg.pose.position.x = data[2 + i*pack_len]
                    msg.pose.position.y = data[3 + i*pack_len]
                    msg.pose.position.z = data[4 + i*pack_len]
                    msg.pose.orientation.x = q_list[0]
                    msg.pose.orientation.y = q_list[1]
                    msg.pose.orientation.z = q_list[2]
                    msg.pose.orientation.w = q_list[3]
                    msgs_list.append(msg)
                     
                elif isinstance(topic, Odometry):
                    q_list = quaternion_from_euler(data[5 + i*pack_len], data[6 + i*pack_len], data[7 + i*pack_len])
                    msg = Odometry()
                    msg.header.stamp = rospy.Time.from_sec(data[1 + i*pack_len])
                    msg.pose.pose.position.x = data[2 + i*pack_len]
                    msg.pose.pose.position.y = data[3 + i*pack_len]
                    msg.pose.pose.position.z = data[4 + i*pack_len]
                    msg.pose.pose.orientation.x = q_list[0]
                    msg.pose.pose.orientation.y = q_list[1]
                    msg.pose.pose.orientation.z = q_list[2]
                    msg.pose.pose.orientation.w = q_list[3]
                    msgs_list.append(msg)
                    
                else:
                    raise KeyError('Package for GloraPackage is not existed.')
            return (idbytes_list, msgs_list, data_list)

        except:
            print("data currupt")
            return (None, None, None)

    def combine(self, gpack):
        if not isinstance(gpack, GloraPackage) or len(gpack.pack) == 0:
            raise KeyError('Parameter \'gpack\' need to be GloraPackage.')

        if len(self.pack) != 0:
            tmp_pack = copy.deepcopy(gpack)

            Lnum_of_info = (len(self.pack)) // 33
            Rnum_of_info = (len(tmp_pack)) // 33

            if Lnum_of_info + Rnum_of_info > 6:
                raise KeyError('Package len is beyond the limit(5).')

            
            self.pack.extend(tmp_pack.pack)
            #self.generate_checksum_()

        else:
            self.pack = copy.deepcopy(gpack.pack)

        return self
