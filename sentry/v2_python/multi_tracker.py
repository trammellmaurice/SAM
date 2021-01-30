import cv2
import sys, os
import random

video = cv2.VideoCapture(0)

# Exit if video not opened.
if not video.isOpened():
    print("Could not open video")
    sys.exit()

# Read first frame.
ok, frame = video.read()
if not ok:
    print('Cannot read video file')
    sys.exit()
#  Create MultiTracker object
multiTracker = cv2.MultiTracker_create()

# Get ROIs
rois = cv2.selectROIs('SELECT_ROI',frame,False)
for roi in rois:
    multiTracker.add(cv2.TrackerMOSSE_create(), frame, tuple(roi))


# Process video and track
while video.isOpened():
    ok, frame = video.read()
    if not ok:
        break

    # get updated rois
    ok, rois = multiTracker.update(frame)

    # draw rois on frame
    for i, newROI in enumerate(rois):
        p1 = (int(newROI[0]), int(newROI[1]))
        p2 = (int(newROI[0] + newROI[2]), int(newROI[1] + newROI[3]))
        cv2.rectangle(frame,p1,p2,(255,0,0), 2, 1)

    # show frame
    cv2.imshow('TRACKER',frame)
    # quit on ESC
    if cv2.waitKey(1) & 0xFF == 27:
        break
