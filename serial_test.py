import Jetson.GPIO as GPIO 
import time

# pin definitions
GPIO.setmode(GPIO.BOARD)

#7 = down
#12 = up
# 13 = fire
# setup channel
channels = [7,11,12,13]

DOWN, UP, FIRE = 7,12,13
GPIO.setup(FIRE,GPIO.OUT,initial=GPIO.LOW)  


GPIO.output(FIRE,GPIO.HIGH)

time.sleep(0.5)

GPIO.output(FIRE,GPIO.LOW)

GPIO.cleanup()
