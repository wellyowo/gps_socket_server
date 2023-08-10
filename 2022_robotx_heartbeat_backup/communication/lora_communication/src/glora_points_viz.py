#!/usr/bin/env python
from __future__ import print_function

import roslib
import numpy as np
import sys
import rospy
import cv2
from std_msgs.msg import String, Header, Float32MultiArray, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class class_node:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic",Image, queue_size = 1)

    self.bridge = CvBridge()
    self.glora_point_sub = rospy.Subscriber("/duckiefloat/glora_points", Float32MultiArray, self.callback)
    

  def callback(self,data):
    self.img = np.zeros((200, 200, 1), dtype="uint8")
    #fake_data = [-4, 2, -3, 2, -2, 2, -1, 2, 2, 2, 3, 2, 4, 2, 5, 2]
    for i in [0, 2, 4, 6, 8, 10, 12, 14]:
        cv2.circle(self.img, (int(data.data[i]*20)+100, int(data.data[i+1]*40)), 1, 255, -1, 8)

    cv2.flip(self.img, 0, self.img)
    #cv2.imshow("Image window", self.img)
    #cv2.waitKey(3)
    #print ("cb")
    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.img, "mono8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('glora_point_node', anonymous=True)
  ic = class_node()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
