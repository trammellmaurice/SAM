#!/usr/bin/env python3
LOCAL = False
ENGAGE_TIME = 20
REDETECT_TIME = 100

import cv2
import math
import random
import sys
import numpy as np

import rospy
from std_msgs.msg import String

# Detection libraries
if not LOCAL:
    import jetson.inference
    import jetson.utils
    import time

# Set up camera input
if not LOCAL:
    video = cv2.VideoCapture(0)

else:
    video = cv2.VideoCapture(0)

# Exit if video not opened.
if not video.isOpened():
    print("Could not open video")
    sys.exit()

if not LOCAL:
    # Set up detection network default SSD-Mobilenet-V2
    net = jetson.inference.detectNet("ssd-mobilenet-v2", 0.4)
    # set up publisher
    pub = rospy.Publisher('auto',String,queue_size=1)
    rospy.init_node('jetson_ai',anonymous=True)
    rate = rospy.Rate(75)

# read in a few frames
for i in range(0,10):
     ok, frame = video.read()

while video.isOpened():
    if not LOCAL:
        # Create MultiTracker object
        multiTracker = cv2.legacy.MultiTracker_create()
    else:
        # Create MultiTracker object
        multiTracker = cv2.MultiTracker_create()
    """
    DETECT TARGETS
    """
    if not LOCAL:
        detections = []
        init_time = time.time()
        while not detections:
            # time for checking signals            
            current_time = time.time()
            if (current_time-init_time) > 10:
                rospy.loginfo("CHECKIN")
                pub.publish("AUTOPILOT OPERATIONAL")  
                init_time = time.time()
        
            # Read first frame.
            ok, frame = video.read()
            # detection
            img = jetson.utils.cudaFromNumpy(frame)
            detections = net.Detect(img)
            if detections and len(detections) > 3:
                detections = detections[0:3]
             # show frame
            cv2.imshow('TURRET',frame)

            # quit on ESC
            if cv2.waitKey(1) & 0xFF == 27:
                sys.exit()

        rois = [(detection.Left,detection.Top,detection.Width,detection.Height) for detection in detections]
           
        for roi in rois:
            multiTracker.add(cv2.legacy.TrackerMOSSE_create(), frame, tuple(roi))
            #multiTracker.add(cv2.legacy.TrackerKCF_create(), frame, tuple(roi))
    else:
        # Read first frame.
        ok, frame = video.read()
        if not ok:
            print('Cannot read video file')
            sys.exit()
        # Get ROIs manually for LOCALing
        rois = cv2.selectROIs('SELECT_ROI',frame,False)
        for roi in rois:
            multiTracker.add(cv2.TrackerMOSSE_create(), frame, tuple(roi))


    """
    TRACKING LOOP
    """
    # Process video and track
    PRIMARY_TARGET = 0
    LOCK = ENGAGE_TIME
    CLOCK = REDETECT_TIME

    while video.isOpened() and CLOCK > 1:
        ok, frame = video.read()
        if not ok:
            break

         # Start timer
        timer = cv2.getTickCount()

        if LOCK == 0:
            PRIMARY_TARGET = (PRIMARY_TARGET + 1) % len(rois)
            LOCK = ENGAGE_TIME

        height, width, channels = frame.shape

        # get updated rois
        ok, rois = multiTracker.update(frame)


        # Calculate Frames per second (FPS)
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);


        # draw crosshair
        x = round(width/2)
        y = round(height/2)
        cv2.line(frame, (x-10,y), (x+10,y), (0,0,255), 2)
        cv2.line(frame, (x,y-10), (x,y+10), (0,0,255), 2)

        target = None
        shoot = False



        # draw rois on frame

        if not ok:
            CLOCK = 0
            break

        for index, newROI in enumerate(rois):
            if index == PRIMARY_TARGET:
                p1 = (int(newROI[0]), int(newROI[1]))
                p2 = (int(newROI[0] + newROI[2]), int(newROI[1] + newROI[3]))
                target = (round((p1[0]+p2[0])/2),round((p1[1]+p2[1])/2))
            else:
                p1 = (int(newROI[0]), int(newROI[1]))
                p2 = (int(newROI[0] + newROI[2]), int(newROI[1] + newROI[3]))
            if index == PRIMARY_TARGET:
                # draw boxes
                if  newROI[0] < x < newROI[0] + newROI[2] and newROI[1] < y < newROI[1] + newROI[3]:
                    shoot = True
                    cv2.rectangle(frame,p1,p2,(0,0,255), 2, 1)
                    LOCK-=1 # decrement lock for being on target
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

        # calculate vector to target center
        vx = x - target[0]
        vy = y - target[1]

        # message transmission
        if shoot:
            rospy.loginfo("FIRE")
            pub.publish("FIRE")
            rate.sleep()
            pub.publish("FSTOP")

        # up, left = + +
        if vx > 10:
            rospy.loginfo("LEFT")
            pub.publish("LEFT")  
            rate.sleep()
            pub.publish("LSTOP") 
            
        elif vx < -10:
            rospy.loginfo("RIGHT")
            pub.publish("RIGHT")  
            rate.sleep()
            pub.publish("RSTOP")  

        if vy > 10:    
            rospy.loginfo("UP")
            pub.publish("UP")
            rate.sleep()
            pub.publish("USTOP") 

        elif vy < -10: 
            rospy.loginfo("DOWN")
            pub.publish("DOWN")
            rate.sleep()
            pub.publish("DSTOP") 

 

        # draw vector on frame
        frame = cv2.line(frame,(x,y),(target[0],target[1]),(0,0,255),2)

        # Display FPS on frame
        cv2.putText(frame, "FPS : " + str(int(fps)), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2)

        # Display vector
        cv2.putText(frame, "MOVDIR X : " + str(int(vx)), (50,100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2)
        cv2.putText(frame, "MOVDIR Y : " + str(int(vy)), (50,150), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0,0,255), 2)

        # show frame
        cv2.imshow('TURRET',frame)
        CLOCK-=1
        # quit on ESC
        if cv2.waitKey(1) & 0xFF == 27:
            sys.exit()
