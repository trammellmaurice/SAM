#!/usr/bin/env python

# system library
import math
import random
import sys
import time

# image library
import cv2
import numpy as np

# ros libraries
# import rospy
# from std_msgs.msg import String

# inference libraries
# import jetson.inference
# import jetson.utils

# # set up camera (use open cv)
video = cv2.VideoCapture(0)

while video.isOpened():
    # READ NEW FRAMES
    ok, frame = video.read()
    if not ok:
        break

    # DISPLAY FRAME
    cv2.imshow('TURRET',frame)
    # quit on ESC
    if cv2.waitKey(1) & 0xFF == 27:
        sys.exit()

# print(cv2.__version__)
