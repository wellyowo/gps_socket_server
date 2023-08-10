#!/usr/bin/env python
import serial
import rospy
import struct
import math
import time
import copy
import random
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Point, Quaternion
from subt_msgs.msg import SubTInfo, ArtifactPose, ArtifactPoseArray, AnchorBallDetection
# Python package
from subt_glora import GloraPackage

PROBABILITY_FOR_ARTIFACT = 0.5
PROBABILITY_FOR_ANCHOR = 0.01
NOISE_PROBABILITY = 0.1
ROBOT_STEP_SIZE = 0.005
ARTIFACTS = ['backpack', 'survivor', 'vent', 'phone', 'CO2']
ARTIFACT_SEARCH_SAMPLING_RATE = 10
ARTIFACT_PUBLISH_ONESHOT = True

def isclose(a, b, rel_tol=1e-09, abs_tol=0.01):
    return abs(a-b) <= max(rel_tol * max(abs(a), abs(b)), abs_tol)

class MsgGenerator(object):
    """docstring for MsgGenerator"""
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        # Vehicle identification
        self.veh = 'husky1'
        #try: self.veh = rospy.get_param('veh')
        #except KeyError:
        #    rospy.logerr('You need to assign \'veh\' for the vehicle name.')
        #    exit(-1)

        # Publisher
        self.pub_subt = rospy.Publisher('husky1/subt_info', SubTInfo, queue_size=1)
        self.pub_robot_pose = rospy.Publisher('robot_pose', PoseStamped, queue_size=1)

        self.fake_robot_pose = Pose(position=Point(random.randint(-10, 10), random.randint(-10, 10), 0))



        self.fake_subt_msg = SubTInfo(header=Header(stamp=rospy.Time.now(), frame_id='slam_odom'),
                                    robot_name=self.veh,
                                    robot_pose=self.fake_robot_pose,
                                    #artifacts=,
                                    # anchorballs=,
                                    )

        self.fake_nav_x = random.randint(0, 100)/100 * random.choice([1, -1])
        self.fake_nav_y = random.randint(0, 100)/100 * random.choice([1, -1])
        rospy.Timer(rospy.Duration(2), self.timer_send_cb)
        # rospy.Timer(rospy.Duration(1/ARTIFACT_SEARCH_SAMPLING_RATE), self.artifact_search_cb)


    def timer_send_cb(self, event):
        # Robot pose
        (q_x, q_y, q_z, q_w) = quaternion_from_euler(0, 0, math.atan2(self.fake_nav_y, self.fake_nav_x))
        self.fake_robot_pose.orientation = Quaternion(x=q_x, y=q_y, z=q_z, w=q_w)
        print(self.fake_nav_x, self.fake_nav_y)
        if isclose(self.fake_nav_x, 0) and isclose(self.fake_nav_y, 0):
            self.fake_nav_x = random.randint(0, 100)/100 * random.choice([1, -1])
            self.fake_nav_y = random.randint(0, 100)/100 * random.choice([1, -1])
        elif isclose(self.fake_nav_x, 0):
            self.fake_robot_pose.position.y += ROBOT_STEP_SIZE*self.fake_nav_y/abs(self.fake_nav_y)
            self.fake_nav_y -= ROBOT_STEP_SIZE*self.fake_nav_y/abs(self.fake_nav_y)
        elif isclose(self.fake_nav_y, 0): 
            self.fake_robot_pose.position.x += ROBOT_STEP_SIZE*self.fake_nav_x/abs(self.fake_nav_x)
            self.fake_nav_x -= ROBOT_STEP_SIZE*self.fake_nav_x/abs(self.fake_nav_x)
        else:
            self.fake_robot_pose.position.x += ROBOT_STEP_SIZE*self.fake_nav_x/abs(self.fake_nav_x)
            self.fake_robot_pose.position.y += ROBOT_STEP_SIZE*self.fake_nav_y/abs(self.fake_nav_y)
            self.fake_nav_x -= ROBOT_STEP_SIZE*self.fake_nav_x/abs(self.fake_nav_x)
            self.fake_nav_y -= ROBOT_STEP_SIZE*self.fake_nav_y/abs(self.fake_nav_y)

        # Artifact 
        fake_arti = ArtifactPoseArray()
        for i in range(len(ARTIFACTS)):
            if random.random() <= PROBABILITY_FOR_ARTIFACT + PROBABILITY_FOR_ARTIFACT*NOISE_PROBABILITY*random.uniform(-1, 1):
                fake_arti.pose_array.append(ArtifactPose(Class=ARTIFACTS[i], pose=self.fake_robot_pose))
                fake_arti.count += 1
        self.fake_subt_msg.artifacts = fake_arti


        self.pub_subt.publish(self.fake_subt_msg)
        self.pub_robot_pose.publish(PoseStamped(header=Header(stamp=rospy.Time.now(), frame_id='map'),
                                                pose=self.fake_robot_pose))

    def on_shutdown(self):
        rospy.loginfo("shutting down [%s]" % (self.node_name))



if __name__ == "__main__":
    rospy.init_node("fake_subt_info_node", anonymous=False)
    node = MsgGenerator()
    rospy.on_shutdown(node.on_shutdown)
    rospy.spin()
