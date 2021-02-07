import inputs
import cv2
import Jetson.GPIO as GPIO
import time
import sys

ARMED = False
SAM = True

# pin definitions
GPIO.setmode(GPIO.BOARD)

# set up GPIO
ALL = [7,12,13,19,21]
DOWN, UP, FIRE, RIGHT, LEFT= 7,12,13,19,21
GPIO.setup(ALL,GPIO.OUT,initial=GPIO.LOW)

video = cv2.VideoCapture(1)

# Exit if video not opened.
if not video.isOpened():
    print("Could not open video")
    sys.exit()

# read in a few frames
for i in range(0,10):
     ok, frame = video.read()

while video.isOpened():
    GPIO.output(ALL,GPIO.LOW)
    time.sleep(0.01)
    try:
        events = get_gamepad()
        for event in events:
            if event.code == "BTN_BASE3" and event.state == 1:
                SAM = False
                print("MANUAL CONTROL ENGAGE")
            elif event.code == "BTN_BASE" and event.state == 0:
                SAM = True
                print("SAM CONTROL ENGAGE")
            if not SAM:
                if event.code == "BTN_DEAD" and event.state == 1:
                    if not ARMED:
                        ARMED = True
                        print("ARMED")
                    else:
                        ARMED = False
                        print("DISARMED")
                elif event.code == "BTN_PINKIE" and event.state == 1:
                    GPIO.output(RIGHT,GPIO.HIGH)
                    print("RIGHT")
                elif event.code == "BTN_BASE2" and event.state == 1:
                    GPIO.output(LEFT,GPIO.HIGH)
                    print("LEFT")
                elif event.code == "BTN_TOP2" and event.state == 1:
                    GPIO.output(UP,GPIO.HIGH)
                    print("UP")
                elif event.code == "BTN_BASE" and event.state == 1:
                    GPIO.output(DOWN,GPIO.HIGH)
                    print("DOWN")

                elif event.code == "BTN_BASE6" and event.state == 1:
                    if ARMED:
                        GPIO.output(FIRE,GPIO.HIGH)
                        print("FIRE")
                    else:
                        print("MUST ARM")
                time.sleep(0.01)
    except:
        pass
