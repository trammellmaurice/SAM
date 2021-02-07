import cv2
from Target_Queue import *

if 1 == 1:
    # Read video
    video = cv2.VideoCapture(0)

    # Read first frame.
    ok, frame = video.read()

    tq = Target_Queue("blah")
    tq.add_target("person",frame,(256, 128, 106, 65))
    print(tq._targets)
