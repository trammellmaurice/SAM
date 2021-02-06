import Jetson.GPIO as GPIO
import time
# pin definitions
GPIO.setmode(GPIO.BOARD)
ALL = [7,12,13,19,21]
GPIO.setup(ALL,GPIO.OUT,initial=GPIO.LOW)
GPIO.cleanup()
