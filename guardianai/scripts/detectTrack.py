#!/usr/bin/env python

"""
IMPORTS
"""
import sys
import time

import math
import random
import threading

sys.path.remove('/opt/ros/melodic/lib/python2.7/dist-packages') # in order to import cv2 under python3

import cv2

sys.path.append('/opt/ros/melodic/lib/python2.7/dist-packages') # append back in order to import rospy

import numpy as np

import rospy
from std_msgs.msg import String

import jetson.inference
import jetson.utils

"""
FUNCTIONS
"""
def crosshair(frame):
    # GET DIMENSIONS OF FRAME
    height, width, channels = frame.shape

    # DRAW CROSSHAIR ON FRAME
    x = int(round(width/2))
    y = int(round(height/2))
    cv2.line(frame, (x-10,y), (x+10,y), (0,0,255), 2)
    cv2.line(frame, (x,y-10), (x,y+10), (0,0,255), 2)

    return frame


def detect():
    # MAKE A BLANK LIST OF DETECTIONS
    detections = []

    # DETECTION LOOP
    while not detections:
        # GET A FRAME
        ok, frame = video.read()

        # CONVERT TO CUDA IMAGE FOR DETECTION
        img = jetson.utils.cudaFromNumpy(frame)
        detections = net.Detect(img) # DETECT

        # LIMIT DETECTIONS TO 3
        if detections and len(detections) > 3:
            detections = detections[0:3]

        # DRAW CROSSHAIR ON FRAME
        frame = crosshair(frame)

         # SHOW FRAME
        cv2.imshow('TURRET',frame)
        # ESCAPE TO QUIT
        if cv2.waitKey(1) & 0xFF == 27:
            sys.exit()

    global TRACK
    TRACK = True
    return detections

def kill_tracker():
    global TRACK
    TRACK = False
    return

"""
SETUP
"""
ENGAGE_TIME = 20
REDETECT_TIME = 20

# SET UP CAMERA
video = cv2.VideoCapture(0)

# Exit if video not opened.
if not video.isOpened():
    print("Could not open video")
    sys.exit()

# # SET UP DETECTION NETWORK
net = jetson.inference.detectNet("ssd-mobilenet-v2", 0.6)

# INITIALIZE ROSPY NODE
rospy.init_node('guardian_ai',anonymous=True)
rate = rospy.Rate(75) #update at 75 hz

# SET UP ROS PUBLISHERS
commands = rospy.Publisher('autopilot_commands',String,queue_size=1)

# READ JUNK FRAMES
for i in range(0,10):
     ok, frame = video.read()

"""
DETECT TARGETS
"""

# VIDEO STREAM LOOP
while video.isOpened():

    # CREATE A NEW, EMPTY MULTITRACKER
    multiTracker = cv2.MultiTracker_create()

    # DETECT TARGETS AND RETURN LIST
    detections = detect()

    # CONVERT DETECTION TO REGION OF INTEREST FOR TRACKER
    rois = [(detection.Left,detection.Top,detection.Width,detection.Height) for detection in detections]

    # ADD ROIS TO MULTI TRACKER
    for roi in rois:
        multiTracker.add(cv2.TrackerMOSSE_create(), frame, tuple(roi))

    """
    TRACK TARGETS
    """
    PRIMARY_TARGET = 0

    # SET TIMER FOR REDETECTION
    redetect_timer = threading.Timer(REDETECT_TIME,kill_tracker)
    redetect_timer.start()

    while video.isOpened() and TRACK:
        # READ NEW FRAMES
        ok, frame = video.read()
        if not ok:
            break

        # UPDATE TRACKERS
        ok, rois = multiTracker.update(frame)

        # DRAW CROSSHAIR
        frame = crosshair(frame)

        # GET CORNER POINTS FOR BOXES
        for newROI in rois:
            p1 = (int(newROI[0]), int(newROI[1]))
            p2 = (int(newROI[0] + newROI[2]), int(newROI[1] + newROI[3]))
            cv2.rectangle(frame,p1,p2,(255,0,0), 2, 1)

        # DISPLAY FRAME
        cv2.imshow('TURRET',frame)
        # quit on ESC
        if cv2.waitKey(1) & 0xFF == 27:
            sys.exit()
