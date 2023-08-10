#!/usr/bin/env python3
import rospy
import unittest
import rostest
import numpy as np
from std_msgs.msg import Float32, Int32
from sensor_msgs.msg import  Joy
import struct
import roslib
import pickle
import time
from datetime import datetime
from subt_msgs.msg import GloraPack

class loraTester(unittest.TestCase):
	def setup(self):
		# Setup the node
		rospy.init_node('loraTester_node', anonymous=False)
		# Setup lora device
		try :
			self.sub_goal = rospy.Subscriber("/lora_log", GloraPack, self.log_cb, queue_size=1)

		except:
			self.test_Setup_lora_device = False
		self.assertEqual(self.test_Setup_xbee_device, True, "Tester Setup lora device success")

	def log_cb(self,msg):
		self.test_Setup_lora_device = msg.success
		
if __name__ == '__main__':
	rostest.rosrun('lora_communication', 'loraTester_node', loraTester)