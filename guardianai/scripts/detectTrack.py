#!/usr/bin/env python3

"""
IMPORTS
"""
import sys
import time

import math
import random
import threading

import cv2

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

    return frame,x,y


def detect():
    # MAKE A BLANK LIST OF DETECTIONS
    detections = []
    i = False

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
        frame,trash,trash = crosshair(frame)

         # SHOW FRAME
        cv2.imshow('TURRET',frame)
        # ESCAPE TO QUIT
        if cv2.waitKey(1) & 0xFF == 27:
            sys.exit()
        if not detections and i == False:
            status.publish('i')
            rospy.loginfo("XSTOP")
            commands.publish("XSTOP")
            rospy.loginfo("YSTOP")
            commands.publish("YSTOP")
            i = True

    global TRACK
    TRACK = True
    return detections

def kill_tracker():
    print('DETECT')
    global TRACK
    TRACK = False
    return

"""
SETUP
"""
ENGAGE_TIME = 3
REDETECT_TIME = 10

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
status = rospy.Publisher('status',String,queue_size=1)

# READ JUNK FRAMES
for i in range(0,10):
     ok, frame = video.read()

"""
DETECT TARGETS
"""

# VIDEO STREAM LOOP
while video.isOpened():

    # CREATE A NEW, EMPTY MULTITRACKER
    multiTracker = cv2.legacy.MultiTracker_create()

    # DETECT TARGETS AND RETURN LIST
    detections = detect()

    # ADD ROIS TO MULTI TRACKER
    for detection in detections:
        multiTracker.add(cv2.legacy.TrackerMOSSE_create(), frame, tuple((detection.Left,detection.Top,detection.Width,detection.Height)))

    """
    TRACK TARGETS
    """
    status.publish("a")

    # SET TIMER FOR REDETECTION
    try:
        redetect_timer.join()
    except:
        pass
    redetect_timer = threading.Timer(REDETECT_TIME,kill_tracker)
    redetect_timer.start()

    PRIMARY_TARGET = 0
    LOCK = ENGAGE_TIME

    while video.isOpened() and TRACK:
        # READ NEW FRAMES
        ok, frame = video.read()
        if not ok:
            break

        # PICK NEXT TARGET IF LOCK EXPIRED
        if LOCK == 0:
            PRIMARY_TARGET = (PRIMARY_TARGET + 1) % len(rois)
            LOCK = ENGAGE_TIME

        # UPDATE TRACKERS
        ok, rois = multiTracker.update(frame)

        # DRAW CROSSHAIR
        frame,x,y = crosshair(frame)

        shoot = False
        target = None

        # DRAW TARGETS
        for index, newROI in enumerate(rois):
            p1 = (int(newROI[0]), int(newROI[1]))
            p2 = (int(newROI[0] + newROI[2]), int(newROI[1] + newROI[3]))
            if index == PRIMARY_TARGET:
                target = (round((p1[0]+p2[0])/2),round((p1[1]+p2[1])/2))
                # draw boxes
                if  newROI[0] < x < newROI[0] + newROI[2] and newROI[1] < y < newROI[1] + newROI[3]:
                    shoot = True
                    cv2.rectangle(frame,p1,p2,(0,0,255), 2, 1)
                    LOCK-=1
                else:
                    shoot = False
                    cv2.rectangle(frame,p1,p2,(255,0,0), 2, 1)
            else:
                #draw dots
                mx = round((p1[0]+p2[0])/2)
                my = round((p1[1]+p2[1])/2)
                radius = 5
                color = (255,0,0)
                thick = 2
                frame = cv2.circle(frame, (mx,my),radius,color, thick)

        """
        CALCULATIONS AND FEEDBACK
        """
        # CALCULATE VECTOR
        vx = x - target[0]
        vy = y - target[1]

        # message transmission
        if shoot:
            rospy.loginfo("FIRE")
            commands.publish("FIRE")

        # up, left = + +
        if vx > 20:
            rospy.loginfo("LEFT")
            commands.publish("LEFT")

        elif vx < -20:
            rospy.loginfo("RIGHT")
            commands.publish("RIGHT")

        else:
            rospy.loginfo("XSTOP")
            commands.publish("XSTOP")

        if vy > 20:
            rospy.loginfo("UP")
            commands.publish("UP")

        elif vy < -20:
            rospy.loginfo("DOWN")
            commands.publish("DOWN")

        else:
            rospy.loginfo("YSTOP")
            commands.publish("YSTOP")

        # DISPLAY FRAME
        cv2.imshow('TURRET',frame)
        # quit on ESC
        if cv2.waitKey(1) & 0xFF == 27:
            sys.exit()
