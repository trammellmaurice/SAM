#!/usr/bin/env python3

# system library
import math
import random
import sys
import time
import subprocess

# image library
import cv2
import numpy as np

# ros libraries
import rospy
from std_msgs.msg import String

# INITIALIZE ROSPY NODE
rospy.init_node('debugger',anonymous=True)
status = rospy.Publisher('status',String,queue_size=1)
rate = rospy.Rate(0.2)

while not rospy.is_shutdown():
    process = subprocess.check_output(['rosnode','list'])
    alive = str(process).find('guardian_ai')
    if alive == -1:
        rospy.loginfo(alive)
        status.publish("h")
    rate.sleep()
