#!/usr/bin/env python
import serial
import rospy
import struct
import math
import time
import copy
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header, Float32MultiArray, Bool
from geometry_msgs.msg import PoseStamped, PoseArray
from subt_msgs.msg import SubTInfo, AnchorBallDetection, GloraPack, ArtifactPoseArray, ArtifactPose
from sensor_msgs.msg import Joy

# Python package
from subt_glora import GloraPackage

DEFAULT_PORT = "/dev/ttyUSB1"
DEFAULT_SERIAL_DELAY = 0.08
PULLING_DELAY = 0.2

class Receiver(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        # Setup serial
        try:
            rospy.get_param('~port')
        except KeyError: 
            rospy.loginfo('No ROS param \'port\', set default port= ' + DEFAULT_PORT)

        # Serial fine delay
        try:
            rospy.get_param('~serial_delay')
        except KeyError: 
            rospy.loginfo('No ROS param \'serial_delay\', set default serial fine delay= ' + str(DEFAULT_SERIAL_DELAY))

        self.serial_delay = rospy.get_param("~serial_delay", DEFAULT_SERIAL_DELAY)
        self.timeout = rospy.get_param('~timeout',1.5)
        self.pulling_delay = rospy.get_param('~pulling_delay',PULLING_DELAY)

        self.port = rospy.get_param("~port", DEFAULT_PORT)
        self.baud = rospy.get_param("~baud", 115200)
        self.log = rospy.get_param('~log',False)
        self.num_veh = rospy.get_param('~num_veh',2)
        self.ser = serial.Serial(self.port, self.baud, timeout=3)

        if self.log:
            self.pub_log = rospy.Publisher('/glora_log',GloraPack,queue_size=1)

        #parameters
        self.estops = [False,False,False,False,False,False]
        self.joy_r = 0.0
        self.joy_l = 0.0
        self.joy_a = 0.0
        self.h_msg = [0.0,0.0,0.0]
        self.husky1_auto = False
        self.husky2_auto = False
        self.button1_mode = False
        self.button2_mode = False

        #Subscriptions
        self.sub_joy = rospy.Subscriber("/joy",Joy,self.cb_joy,queue_size=1)

        # Publications
        self.pub_husky1_pose = rospy.Publisher("/husky1/pose_new", PoseStamped, queue_size=1)
        self.pub_husky1_mode = rospy.Publisher("/husky1/mode", Bool, queue_size=1)
        self.pub_husky2_pose = rospy.Publisher("/husky2/pose_new", PoseStamped, queue_size=1)
        self.pub_husky2_mode = rospy.Publisher("/husky2/mode", Bool, queue_size=1)
        self.pub_duckiefloat_point= rospy.Publisher("/duckiefloat/glora_points", Float32MultiArray, queue_size=1)
        
        self.pub_artipose_list = rospy.Publisher("/glora/artifact_global",ArtifactPoseArray,queue_size=1)

        #self.pub_anchorball = rospy.Publisher("/anchorball/info_new", AnchorBallDetection, queue_size=1)
        self.anchorball_infolist = []

        self.timer = rospy.Timer(rospy.Duration(0.01), self.process)


    def process(self,event):
        # ID: husky --> 0x10 ~ 0x20
        # ID: duckiefloat --> 0x30 ~ 0x60
        #rospy.sleep(0.5)
        for idx_veh in range(1, self.num_veh+1):
            #rospy.loginfo("sleep")
            self.pulling_delay = rospy.get_param('~pulling_delay',PULLING_DELAY)
            rospy.sleep(self.pulling_delay)
            start_time = time.time()
            identity_byte = (idx_veh << 4) | self.estops[idx_veh-1]
            send_pack = GloraPackage()
            if idx_veh==1:
                if self.button1_mode:    
                    send_pack.pack = bytearray(struct.pack("1B3f", identity_byte,self.h_msg[0],self.h_msg[1],self.h_msg[2]))
                else:
                    if self.husky1_auto:
                        send_pack.pack = bytearray(struct.pack("1B3f", identity_byte,100,0,0))
                    else:
                        send_pack.pack = bytearray(struct.pack("1B3f", identity_byte,-100,0,0))
            elif idx_veh==2:
                if self.button2_mode:    
                    send_pack.pack = bytearray(struct.pack("1B3f", identity_byte,self.h_msg[0],self.h_msg[1],self.h_msg[2]))
                else:
                    if self.husky2_auto:
                        send_pack.pack = bytearray(struct.pack("1B3f", identity_byte,100,0,0))
                    else:
                        send_pack.pack = bytearray(struct.pack("1B3f", identity_byte,-100,0,0))
            else:
                send_pack.pack = bytearray(struct.pack("1B3f", identity_byte,self.joy_l,self.joy_r, 1.0 if self.estops[2] else 0.0))
            
            send_pack.generate_checksum_()
            #pack.extend(('\r', '\n','\r', '\n','\r', '\n','\r', '\n','\r', '\n','\r'))
            
            log_pack = GloraPack()
            log_pack.packet = ''
            if self.log:
                log_pack.header = Header()
                log_pack.header.stamp = rospy.Time.now()
            # Send cmd to veh
            self.ser.write(send_pack.pack)

            try:
                # Wait for data return from idx_veh
                serial_start_t = rospy.Time.now()
                self.timeout = rospy.get_param('~timeout',1.5)
                while rospy.Time.now() - serial_start_t < rospy.Duration(self.timeout):
                    if self.ser.inWaiting() > 0:
                        break

                # Time delay for serial buffer reading
                time.sleep(self.serial_delay)
                recv_pack = self.ser.read(self.ser.inWaiting())
                # print(len(recv_pack))

                if len(recv_pack) == 0:
                    rospy.logerr('No serial data return from veh:{}'.format(idx_veh))
                    if self.log:
                        log_pack.success = False
                        log_pack.time_elapse = time.time() - start_time
                        self.pub_log.publish(log_pack)
                    continue
                

                if not GloraPackage().check_package(list(struct.unpack(str(len(recv_pack))+'B', recv_pack))):
                    rospy.logerr('Wrong checksum from veh:{} len={}'.format(idx_veh, len(recv_pack)))
                    print(list(struct.unpack(str(len(recv_pack))+'B', recv_pack)))
                    if self.log:
                        log_pack.success = False
                        log_pack.packet = str(len(recv_pack))
                        log_pack.time_elapse = time.time() - start_time
                        self.pub_log.publish(log_pack)
                    continue

                ack_pack = recv_pack
                ack_pack = ack_pack[:9]
                if ack_pack == 'ackackack':
                    rospy.loginfo('veh:{} no data to return'.format(idx_veh))
                    if self.log:
                        log_pack.success = True
                        log_pack.packet = str(len(ack_pack))
                        log_pack.time_elapse = time.time() - start_time
                        self.pub_log.publish(log_pack)
                    continue


                # Decoder
                idbytes_list = [] 
                msgs_list = [] 
                data_list = []

                if idx_veh == 3:
                    #print("len:{},{}".format(len(str(recv_pack[:65])),list(struct.unpack(str(len(recv_pack[:65]))+'B', recv_pack[:65])))
                    idbyte, point_msg = GloraPackage().extract_point(recv_pack[:65])
                    #1B + 16f(64B) = 64B
                    recv_pack = recv_pack[65:]
                    rospy.loginfo('duckiefloat point published')
                    self.pub_duckiefloat_point.publish(point_msg)


                idbytes_list, msgs_list, data_list = GloraPackage().unpack(recv_pack, PoseStamped())
                if idbytes_list == None:
                    rospy.logerr('None of recv_byte return from veh:{}'.format(idx_veh))
                    continue
                elif len(idbytes_list) != len(msgs_list):
                    rospy.logerr('None of recv_byte return from veh:{}'.format(idx_veh))
                    continue
                    


                #for log
                if self.log:
                    log_pack.packet = str(len(recv_pack))
                    log_pack.time_elapse = time.time() - start_time
                    log_pack.success = True
                    self.pub_log.publish(log_pack)


                arti_list = ArtifactPoseArray()
                arti_list.header = Header()
                arti_list.header.stamp = rospy.Time.now()
                arti_list.camera = 'Nan'
                arti_list.count = 0


                # Iterate to publish msg for glora data
                for i in range(len(idbytes_list)):
                    # print('Msg get, id_byte= ' + hex(idbytes_list[i]))

                    #
                    # idbyte 0xX0: veh -> map
                    #
                    if idbytes_list[i] & 0x0f == 0x00:
                        msgs_list[i].header.frame_id = "map"
                        if idx_veh == 1:
                            self.pub_husky1_pose.publish(msgs_list[i])
                            mode_msg = Bool()
                            mode_msg.data = True if data_list[i] == 1 else False
                            self.pub_husky1_mode.publish(mode_msg)
                            if data_list[i] == 1:
                                rospy.logerr('husky1 is waiting further instruction')
                            rospy.loginfo(hex(idbytes_list[i]) + ': Husky1 pose published.')
                        elif idx_veh == 2:
                            self.pub_husky2_pose.publish(msgs_list[i])
                            mode_msg = Bool()
                            mode_msg.data = True if data_list[i] == 1 else False
                            self.pub_husky2_mode.publish(mode_msg)
                            if data_list[i] == 1:
                                rospy.logerr('husky2 is waiting further instruction')
                            rospy.loginfo(hex(idbytes_list[i]) + ': Husky2 pose published.')
                        elif idx_veh == 3:
                            self.pub_duckiefloat_pose.publish(msgs_list[i])
                            rospy.loginfo(hex(idbytes_list[i]) + ': Duckiefloat pose published.')
                            
                    #
                    # idbyte 0xXA~0xXE: artifact -> map
                    #
                    elif idbytes_list[i] & 0x0f >= 0x0A:
                        pose = copy.deepcopy(msgs_list[i].pose)                  
                        if idbytes_list[i] & 0x0f == 0x0A:                    # Extinguisher
                            artipose = ArtifactPose()
                            artipose.Class = 'backpack'
                            artipose.appear_count = data_list[i]
                            artipose.probability = 100
                            artipose.pose = pose
                            arti_list.count += 1
                            arti_list.pose_array.append(artipose)
                            rospy.loginfo(hex(idbytes_list[i]) + ': Backpack PoseArray published')

                        elif idbytes_list[i] & 0x0f == 0x0B:                  # Drill
                            artipose = ArtifactPose()
                            artipose.Class = 'survivor'
                            artipose.appear_count = data_list[i]
                            artipose.probability = 100
                            artipose.pose = pose
                            arti_list.count += 1
                            arti_list.pose_array.append(artipose)
                            rospy.loginfo(hex(idbytes_list[i]) + ': Survivor PoseArray published')

                        elif idbytes_list[i] & 0x0f == 0x0C:                  # Survivor
                            artipose = ArtifactPose()
                            artipose.Class = 'vent'
                            artipose.appear_count = data_list[i]
                            artipose.probability = 100
                            artipose.pose = pose
                            arti_list.count += 1
                            arti_list.pose_array.append(artipose)
                            rospy.loginfo(hex(idbytes_list[i]) + ': Vent PoseArray published')

                        elif idbytes_list[i] & 0x0f == 0x0D:                  # Red backpack
                            artipose = ArtifactPose()
                            artipose.Class = 'phone'
                            artipose.appear_count = data_list[i]
                            artipose.probability = 100
                            artipose.pose = pose
                            arti_list.count += 1
                            arti_list.pose_array.append(artipose)
                            rospy.loginfo(hex(idbytes_list[i]) + ': Phone PoseArray published')

                        elif idbytes_list[i] & 0x0f == 0x0E:                  # Smartphone
                            artipose = ArtifactPose()
                            artipose.Class = 'CO2'
                            artipose.appear_count = data_list[i]
                            artipose.probability = 100
                            artipose.pose = pose
                            arti_list.count += 1
                            arti_list.pose_array.append(artipose)
                            rospy.loginfo(hex(idbytes_list[i]) + ': CO2 PoseArray published')

                    #
                    # idbyte 0xX1~0xX6: anchorball
                    #
                    elif idbytes_list[i] & 0x0f > 0x00 and idbytes_list[i] & 0x0f < 0x0A: 
                        # TODO
                        # AnchorBall info: Identity_byte, pose, timestamp
                        # anchorball_info = [idbytes_list[i], pose, copy.deepcopy(msg.header.stamp)]
                        # self.anchorball_infolist.append(anchorball_info)
                        msg = AnchorBallDetection()
                        #msg.robot_id = idx_veh
                        #msg.anchorball_id = idbytes_list[i] & 0x0f
                        #msg.stamp = copy.deepcopy(msgs_list[i].header.stamp)
                        #msg.pose = copy.deepcopy(msgs_list[i].pose)
                        #self.pub_anchorball.publish(msg)
                        #rospy.loginfo(hex(idbytes_list[i]) + ': AnchorBall{} info published.'.format(idbytes_list[i] & 0x0f))

                self.pub_artipose_list.publish(arti_list)
                    
            except serial.SerialTimeoutException:
                print('veh id:{} read timeout'.format(identity_byte))
                print('time elapse {}'.format(time.time() - start_time))
                continue

            print('time elapse {}'.format(time.time() - start_time))
        
            
    def cb_joy(self, joy_msg):    
        self.joy_r = joy_msg.axes[1]
        self.joy_l = joy_msg.axes[3]

        if joy_msg.axes[7]==1:
            self.estops[3] = not self.estops[3]
            self.button_mode = True
            rospy.logerr('up button husky:{}'.format('activate' if self.estops[3] else 'release'))
            self.h_msg[0] = 1.0 if self.estops[3] else 0.0
        
        elif joy_msg.axes[6]==1:
            self.estops[4] = not self.estops[4]
            self.button_mode = True
            rospy.logerr('left button husky:{}'.format('activate' if self.estops[4] else 'release'))
            self.h_msg[1] = 1.0 if self.estops[4] else 0.0

        elif joy_msg.axes[6]==-1:
            self.estops[5] = not self.estops[5]
            self.button_mode = True
            rospy.logerr('right button husky:{}'.format('activate' if self.estops[5] else 'release'))
            self.h_msg[2] = 1.0 if self.estops[5] else 0.0
        

        # A button
        if (joy_msg.buttons[0] == 1):
            self.estops[2] = not self.estops[2]
            rospy.logerr('Auto duckiefloat:{}'.format('activate' if self.estops[2] else 'release'))
            
        # B button
        elif (joy_msg.buttons[1] == 1):
            rospy.logerr('husky manual')
            self.button_mode = False
            self.husky_auto = False
            #rospy.loginfo('E-STOP DUCKIEFLOAT4:{}'.format('activate' if self.estops[4] else 'release'))


        # X button
        elif (joy_msg.buttons[2] == 1):
            rospy.logerr('husky auto')
            self.button_mode = False
            self.husky_auto = True
            # rospy.loginfo('Auto duckiefloat:{}'.format('activate' if self.estops[3] else 'release'))
            

        # Y button
        elif (joy_msg.buttons[3] == 1):
            self.estops[1] = not self.estops[1]
            rospy.logerr('E-STOP DUCKIEFLOAT1:{}'.format('activate' if self.estops[1] else 'release'))

        # Left back button
        elif (joy_msg.buttons[4] == 1):
            pass
            #rospy.loginfo('left back button')

        # Right back button
        elif (joy_msg.buttons[5] == 1):
            pass
            #rospy.loginfo('right back button')

        # Back button
        elif (joy_msg.buttons[6] == 1):
            pass
            #rospy.loginfo('back button')
            
        # Start button
        elif (joy_msg.buttons[7] == 1):
            self.estops[0] = not self.estops[0]
            rospy.logerr('E-STOP HUSKY:{}'.format('activate' if self.estops[0] else 'release'))

        # Power/middle button
        elif (joy_msg.buttons[8] == 1):
            pass
            #rospy.loginfo('middle button')

        # Left joystick button
        elif (joy_msg.buttons[9] == 1):
            pass
            #rospy.loginfo('left joystick button')

        else:
            pass
            #some_active = sum(joy_msg.buttons) > 0
            #if some_active:
            #    rospy.loginfo('No binding for joy_msg.buttons = %s' % str(joy_msg.buttons))



    def on_shutdown(self):
        rospy.loginfo("shutting down [%s]" %(self.node_name))

if __name__ == "__main__":
    rospy.init_node("glora_host", anonymous=False)
    recv_node = Receiver()
    rospy.on_shutdown(recv_node.on_shutdown)
    rospy.spin()
