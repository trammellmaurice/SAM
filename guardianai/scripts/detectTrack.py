#!/usr/bin/env python3

"""
IMPORTS
"""
import sys
import time

import math
import random

import cv2
import numpy as np

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


"""
SETUP
"""
# SET UP CAMERA
try:
    video = cv2.VideoCapture(0)
except:
    video = cv2.VideoCapture(1)

# # SET UP DETECTION NETWORK
# net = jetson.inference.detectNet("ssd-mobilenet-v2", 0.4)
# SET UP ROS PUBLISHER
pub = rospy.Publisher("image_topic",Image)
# INITIALIZE ROSPY NODE
rospy.init_node('guardian_ai',anonymous=True)
rate = rospy.Rate(75) #update at 75 hz

#SET UP BRIDGE
bridge = CvBridge()

# READ FRAMES
ok, frame = video.read()

try:
    pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
except CvBridgeError as e:
    print(e)
