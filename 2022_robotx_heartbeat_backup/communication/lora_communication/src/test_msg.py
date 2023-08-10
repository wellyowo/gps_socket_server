#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from std_msgs.msg import String

sample_rate = 0.2


def send_msg():

    pub_odo = rospy.Publisher("odom",Odometry,queue_size=1)

    rate = rospy.Rate(sample_rate)

    while not rospy.is_shutdown():
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        pub_odo.publish(odom)
        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("test_msg",anonymous=False)
    send_msg()
