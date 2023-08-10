#!/usr/bin/env python
import serial
import rospy
import struct
import math
import time
import copy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Header, Float32MultiArray
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point
from subt_msgs.msg import SubTInfo, ArtifactPose, AnchorBallDetection, GloraPack
from sensor_msgs.msg import Joy, NavSatFix

# Python package
from subt_glora import GloraPackage

DEFAULT_PORT = "/dev/ttyUSB0"
MAX_NUM_OF_PACKLIST = 5
ROBOT_POSE_SAMPLE_PERIOD = 1


class Sender(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " % (self.node_name))

        self.log = rospy.get_param('~log', False)
        if self.log:
            self.pub_log = rospy.Publisher(
                'glora_log', GloraPack, queue_size=1)

        # Vehicle identification
        self.veh = None
        try:
            self.veh = rospy.get_param('~veh')
        except KeyError:
            rospy.logerr('You need to assign \'veh\' for the vehicle name.')
            exit(-1)

        self.identity_byte = None
        if self.veh.lower() == 'husky1':
            self.identity_byte = 0x10
        if self.veh.lower() == 'husky2':
            self.identity_byte = 0x20
        elif self.veh.lower() == 'duckiefloat1':
            self.identity_byte = 0x30
        elif self.veh.lower() == 'duckiefloat2':
            self.identity_byte = 0x40

        # Setup serial
        try:
            rospy.get_param('~port')
        except KeyError:
            rospy.loginfo(
                'No ROS param \'port\', set default port= ' + DEFAULT_PORT)
        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baud = rospy.get_param("~baud", 115200)
        self.ser = serial.Serial(self.port, self.baud)
        self.estop = 0

        # publisher
        #self.pub_joy = rospy.Publisher("/joy",Joy,queue_size=1)
        #self.pub_estop = rospy.Publisher("e_stop_lora", Bool, queue_size=1)
        self.pub_cmd = rospy.Publisher("glora_cmd",Float32MultiArray, queue_size=1)
        self.pub_husky_mode = rospy.Publisher("husky_mode",Bool,queue_size=1)

        # Subscriber
        # self.sub_odom = rospy.Subscriber("/"+ self.veh + "/odom", PoseStamped, self.odom_cb, queue_size=1)
        self.sub_odom = rospy.Subscriber(
            "subt_info", SubTInfo, self.subt_msg_cb, queue_size=1)
        self.sub_mode = rospy.Subscriber('husky/stop_mode',Bool,self.cb_mode,queue_size=1)

        self.mode = False
        self.pack_list = []
        self.robot_packlist = []
        self.artifact_packlist = []
        self.anchor_packlist = []

        self.timer_start = rospy.Time.now()

        self.timer = rospy.Timer(rospy.Duration(0.01), self.process)
        #self.timer_estop = rospy.Timer(rospy.Duration(0.1), self.cb_pub_estop)
        self.timer_arrange = rospy.Timer(rospy.Duration(ROBOT_POSE_SAMPLE_PERIOD), self.arrange_packet)
    
    def cb_mode(self,msg):
        self.mode = msg.data

    def cb_pub_estop(self, event):
        self.pub_estop.publish(self.estop)

    def process(self, event):
        rospy.sleep(0.1)
        log_success = False

        #tmp_pack = self.ser.readline()

        if self.ser.inWaiting() > 0:
            time.sleep(0.08)
            #recv_pack = self.ser.read(self.ser.inWaiting())
        else:
            return

        tmp_pack = self.ser.read(self.ser.inWaiting())
        if len(tmp_pack) != 18:
            return
        
        cmd_data = []
        try:
            cmd_data = struct.unpack('1B3f2B', tmp_pack)
        except:
            print('unpack fail')

        if cmd_data[0] & 0xf0 == self.identity_byte:
            if not GloraPackage().check_package(list(struct.unpack(str(len(tmp_pack))+'B', tmp_pack))):
                rospy.logerr('Wrong checksum len={}'.format( len(tmp_pack)))
                print(list(struct.unpack(str(len(tmp_pack))+'B', tmp_pack)))
                return

            # check emergency stop
            new_es = cmd_data[0] & 0x0f
            if new_es != self.estop:
                self.estop = new_es
                rospy.loginfo('ESTOP:{}'.format(
                    'activate' if self.estop == 1 else 'release'))
            
            cmd_msg = Float32MultiArray()
            if abs(cmd_data[1]) < 10:
                cmd_msg.data.append(cmd_data[1])
                cmd_msg.data.append(cmd_data[2])
                cmd_msg.data.append(cmd_data[3])
                self.pub_cmd.publish(cmd_msg)
                print('pub: {}, {}, {}'.format(cmd_data[1],cmd_data[2],cmd_data[3]))
            else:
                mode = True if cmd_data[1] > 0 else False
                self.pub_husky_mode.publish(mode)

            # Send glora package
            pack_str = ''
            if len(self.pack_list) > 0:
                log_success = True
                glora_pack = self.pack_list.pop(0)
                glora_pack.generate_checksum_()

                # glora_out
                pack_str = str(len(glora_pack.pack))
                self.ser.write(glora_pack.pack)
                tmp_str = '['
                for i in range((len(glora_pack.pack)) // 33):
                    tmp_str = tmp_str + hex(glora_pack.pack[i*33]) + ', '
                rospy.loginfo(
                    'Msg Sent, len= ' + str(len(glora_pack.pack)) + ', id_byte= ' + tmp_str + ']')
            else:
                log_success = False
                rospy.loginfo(self.veh + ' has no glora package to send.')
                ack_pack = GloraPackage()
                ack_pack.pack = bytearray()
                ack_pack.pack.extend('ackackack')
                ack_pack.generate_checksum_()
                self.ser.write(ack_pack.pack)

            if self.log:
                log_pack = GloraPack()
                log_pack.header = Header()
                log_pack.header.stamp = rospy.Time.now()
                log_pack.packet = pack_str
                log_pack.success = log_success
                self.pub_log.publish(log_pack)

    def subt_msg_cb(self, msg):
        # now = rodata_listspy.Time.now()
        if msg.robot_name.lower() != self.veh.lower():
            rospy.logerr('Robot name from msg is wrong.')
            exit(-1)

        # Robot pose:
        gpack = GloraPackage(self.identity_byte | 0x00, msg=copy.deepcopy(msg.robot_pose),
                             time_stamp=copy.deepcopy(msg.header.stamp),data= 1 if self.mode else 0)

        if len(self.robot_packlist) <= MAX_NUM_OF_PACKLIST:
            self.robot_packlist.append(gpack)
        else:
            _ = self.robot_packlist.pop(0)
            self.robot_packlist.append(gpack)

        # Artifact pose:
        for i in range(msg.artifacts.count):
            pose_msg = copy.deepcopy(msg.artifacts.pose_array[i].pose)
            stamp = copy.deepcopy(msg.artifacts.header.stamp)
            appear_count = msg.artifacts.pose_array[i].appear_count
            if msg.artifacts.pose_array[i].Class == 'backpack':
                gpack = GloraPackage(self.identity_byte |
                                     0x0A, msg=pose_msg, time_stamp=stamp, data=appear_count)

            elif msg.artifacts.pose_array[i].Class == 'survivor':
                gpack = GloraPackage(self.identity_byte |
                                     0x0B, msg=pose_msg, time_stamp=stamp, data=appear_count)

            elif msg.artifacts.pose_array[i].Class == 'vent':
                gpack = GloraPackage(self.identity_byte |
                                     0x0C, msg=pose_msg, time_stamp=stamp, data=appear_count)

            elif msg.artifacts.pose_array[i].Class == 'phone':
                gpack = GloraPackage(self.identity_byte |
                                     0x0D, msg=pose_msg, time_stamp=stamp, data=appear_count)

            elif msg.artifacts.pose_array[i].Class == 'CO2':
                gpack = GloraPackage(self.identity_byte |
                                     0x0E, msg=pose_msg, time_stamp=stamp, data=appear_count)

            else:
                rospy.logerr(
                    'Class:' + msg.artifacts.pose_array[i].Class + ' is unknown.')
                return

            if len(self.artifact_packlist) <= MAX_NUM_OF_PACKLIST:
                self.artifact_packlist.append(gpack)
            else:
                _ = self.artifact_packlist.pop(0)
                self.artifact_packlist.append(gpack)

        # print('total artifact detection number{}'.format(len(self.artifact_packlist)))

        # AnchorBall info
        #if msg.has_ball:
        #    anchor_id = msg.anchorballs.anchorball_id
        #    pose_msg = copy.deepcopy(msg.anchorballs.pose)
        #    stamp = copy.deepcopy(msg.anchorballs.stamp)

        #    if anchor_id == 0 or anchor_id >= 0x0A:
        #        rospy.logerr('AnchorBall id:{} is unknown.'.format(anchor_id))
        #    else:
        #        gpack = GloraPackage(self.identity_byte |
        #                             anchor_id, msg=pose_msg, time_stamp=stamp)
        #        if len(self.anchor_packlist) <= MAX_NUM_OF_PACKLIST:
        #            self.anchor_packlist.append(gpack)
        #        else:
        #            _ = self.anchor_packlist.pop(0)
        #            self.anchor_packlist.append(gpack)

        # print('robot_pose_len={}\nartifact_len={}\nanchor_len={}\npacklist_len={}\n=============='.format(
        #         len(self.robot_packlist),
        #         len(self.artifact_packlist),
        #         len(self.anchor_packlist),
        #         len(self.pack_list)))

        # Collect pack to pack_list
        # 1. Reserve at least 1 pack size for robot_pose anyway.
        # 2. Reserve at least 1 pack size for anchor_pose if it is existed.
        # 3. Which pack have greater length, anchor_pose or artifact_pose?
        # 4. Due to (3), fill pack_list by the pack which has greater length.
        # 5. If pack_list has space, fill it by anchor_pose or robot_pose

    def arrange_packet(self,event):
        while len(self.pack_list) > MAX_NUM_OF_PACKLIST:
            _ = self.pack_list.pop(0)

        if len(self.pack_list) <= MAX_NUM_OF_PACKLIST:
            gpack = GloraPackage()

            # (1)
            if len(self.robot_packlist) > 0:
                gpack = self.robot_packlist.pop(0)

            # (2)
            #if len(self.anchor_packlist) > 0:
            #    if gpack != None:
            #        gpack.combine(self.anchor_packlist.pop(0))
            #    else:
            #        gpack = self.anchor_packlist.pop(0)

            # (3, 4)
            if len(self.artifact_packlist) > len(self.anchor_packlist):
                for i in range(len(self.artifact_packlist)):
                    if (len(gpack.pack))//33 >= 6:
                        break
                if len(self.artifact_packlist) > 0:
                    if gpack != None:
                        gpack.combine(self.artifact_packlist.pop(0))
                    else:
                        gpack = self.artifact_packlist.pop(0)
            else:
                for i in range(len(self.anchor_packlist)):
                    if (len(gpack.pack))//33 >= 6:
                        break
                    if len(self.anchor_packlist) > 0:
                        if gpack != None:
                            gpack.combine(self.anchor_packlist.pop(0))
                        else:
                            gpack = self.anchor_packlist.pop(0)

            # (5)
            if gpack != None:
                if (len(gpack.pack))//33 < 6 and len(self.anchor_packlist) > 0:
                    for i in range(len(self.anchor_packlist)):
                        if (len(gpack.pack))//33 >= 6:
                            break
                        if len(self.anchor_packlist) > 0:
                            gpack.combine(self.anchor_packlist.pop(0))
                elif (len(gpack.pack))//33 < 6 and len(self.artifact_packlist) > 0:
                    for i in range(len(self.artifact_packlist)):
                        if (len(gpack.pack))//33 >= 6:
                            break
                        if len(self.artifact_packlist) > 0:
                            gpack.combine(self.artifact_packlist.pop(0))

            if len(gpack.pack) != 0:
                self.pack_list.append(gpack)
                self.timer_start = rospy.Time.now()

    def on_shutdown(self):
        rospy.loginfo("shutting down [%s]" % (self.node_name))


if __name__ == "__main__":
    rospy.init_node("glora_client", anonymous=False)
    trans_node = Sender()
    rospy.on_shutdown(trans_node.on_shutdown)
    rospy.spin()
