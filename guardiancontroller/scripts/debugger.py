#!/usr/bin/env python

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
# SET UP ROS PUBLISHERS
status = rospy.Publisher('status',String,queue_size=1)
commands = rospy.Publisher('autopilot_commands',String,queue_size=1)
rate = rospy.Rate(0.2)

while not rospy.is_shutdown():
    process = subprocess.check_output(['rosnode','list'])
    alive = str(process).find('guardian_ai')
    if alive == -1:
        rospy.loginfo(alive)
        status.publish("h")
        commands.publish("XSTOP")
        commands.publish("YSTOP")
    rate.sleep()
