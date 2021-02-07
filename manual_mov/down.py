import Jetson.GPIO as GPIO 
import time

# pin definitions
GPIO.setmode(GPIO.BOARD)

#7 = down
#12 = up
# 13 = fire
# setup channel
channels = [7,12,13,19,21]

DOWN, UP, FIRE,RIGHT,LEFT = 7,12,13,19,21
GPIO.setup(channels,GPIO.OUT,initial=GPIO.LOW)  


GPIO.output(DOWN,GPIO.HIGH)

time.sleep(0.1)

GPIO.output(DOWN,GPIO.LOW)

GPIO.cleanup()
