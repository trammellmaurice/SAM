TEST = True
ENGAGE_TIME = 30

import cv2
import math
import random

# Detection libraries
if not TEST:
    import jetson.inference
    import jetson.utils

# Set up camera input
if not TEST:
    video = cv2.VideoCapture(2)
else:
    video = cv2.VideoCapture(0)

# Create MultiTracker object
multiTracker = cv2.MultiTracker_create()

# Exit if video not opened.
if not video.isOpened():
    print("Could not open video")
    sys.exit()

if not TEST:
    # Set up detection network default SSD-Mobilenet-V2
    net = jetson.inference.detectNet("ssd-mobilenet-v2", 0.5)

# Read first frame.
ok, frame = video.read()
if not ok:
    print('Cannot read video file')
    sys.exit()

"""
DETECT TARGETS
"""
if not TEST:
    # detection
    img = jetson.utils.cudaFromNumpy(frame)
    detections = net.Detect(img)
    rois = [(detection.Left,detection.Right,detection.Width,detection.Height) for detection in detections]
    for roi in rois:
        multiTracker.add(cv2.legacy.TrackerMOSSE_create(), frame, tuple(roi))
else:
    # Get ROIs manually for testing
    rois = cv2.selectROIs('SELECT_ROI',frame,False)
    for roi in rois:
        multiTracker.add(cv2.TrackerMOSSE_create(), frame, tuple(roi))

"""
TRACKING LOOP
"""
# Process video and track
PRIMARY_TARGET = 0
LOCK = ENGAGE_TIME

while video.isOpened():
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
        print([0])
    else:
        # up, left = + +
        tx = 1 if vx > 0 else 0
        ty = 1 if vy > 0 else 0
        print([tx,ty])

    # draw vector on frame
    frame = cv2.line(frame,(x,y),(target[0],target[1]),(0,0,255),2)

    # Display FPS on frame
    cv2.putText(frame, "FPS : " + str(int(fps)), (50,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)

    # Display vector
    cv2.putText(frame, "MOVDIR X : " + str(int(vx)), (50,100), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)
    cv2.putText(frame, "MOVDIR Y : " + str(int(vy)), (50,150), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50), 2)

    # show frame
    cv2.imshow('TURRET',frame)
    # quit on ESC
    if cv2.waitKey(1) & 0xFF == 27:
        break
