from inputs import get_gamepad
import cv2
import Jetson.GPIO as GPIO
import time

ARMED = False
SAM = True

# pin definitions
GPIO.setmode(GPIO.BOARD)

# set up GPIO
ALL = [7,12,13,19,21]
DOWN, UP, FIRE, RIGHT, LEFT= 7,12,13,19,21
GPIO.setup(ALL,GPIO.OUT,initial=GPIO.LOW)

video = cv2.VideoCapture(0)

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
    events = get_gamepad()
    for event in events:
        # if event.ev_type != "Sync" and event.ev_type != "Absolute":
        #     print(event.ev_type, event.code, event.state)
        if event.code == "BTN_TL2" and event.state == 1:
            SAM = False
            print("MANUAL CONTROL ENGAGE")
        elif event.code == "BTN_TL2" and event.state == 0:
            SAM = True
            print("SAM CONTROL ENGAGE")
        if not SAM:
            if event.code == "BTN_WEST" and event.state == 1:
                ARMED = True
                print("ARMED")
            elif event.code == "BTN_NORTH" and event.state == 1:
                ARMED = False
                print("DISARMED")
            elif event.code == "BTN_DPAD_RIGHT" and event.state == 1:
                GPIO.output(RIGHT,GPIO.HIGH)
                print("RIGHT")
            elif event.code == "BTN_DPAD_LEFT" and event.state == 1:
                GPIO.output(LEFT,GPIO.HIGH)
                print("LEFT")
            elif event.code == "BTN_DPAD_UP" and event.state == 1:
                GPIO.output(UP,GPIO.HIGH)
                print("UP")
            elif event.code == "BTN_DPAD_DOWN" and event.state == 1:
                GPIO.output(DOWN,GPIO.HIGH)
                print("DOWN")

            elif event.code == "BTN_TR" and event.state == 1:
                if ARMED:
                    GPIO.output(FIRE,GPIO.HIGH)
                    print("FIRE")
                else:
                    print("MUST ARM")
            time.sleep(0.01)
