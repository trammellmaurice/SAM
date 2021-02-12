#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String 

import RPi.GPIO as GPIO

import os 


SAM = True
MUSIC = False

# pin definitions
GPIO.setmode(GPIO.BOARD)

# set up GPIO
ALL = [3,7,12,18,23]
FIRE,UP,DOWN,RIGHT,LEFT = 3,7,12,18,23
GPIO.setup(ALL,GPIO.OUT,initial=GPIO.LOW)

def callbackA(auto):
    global SAM
    if SAM:
        if auto.data == "RIGHT":
            GPIO.output(RIGHT,GPIO.HIGH)
        elif auto.data == "RSTOP":
            GPIO.output(RIGHT,GPIO.LOW)

        if auto.data == "LEFT":
            GPIO.output(LEFT,GPIO.HIGH)
        elif auto.data == "LSTOP":
            GPIO.output(LEFT,GPIO.LOW)

    
        if auto.data == "UP":
            GPIO.output(UP,GPIO.HIGH)
        elif auto.data == "USTOP":
            GPIO.output(UP,GPIO.LOW)
            

        if auto.data == "DOWN":
            GPIO.output(DOWN,GPIO.HIGH)
        elif auto.data == "DSTOP":
            GPIO.output(DOWN,GPIO.LOW)

        
        if auto.data == "FIRE":
            GPIO.output(FIRE,GPIO.HIGH)
            global MUSIC
            if not MUSIC:
                os.system('mpg321 /media/pi/BE82-0E13/*.mp3 &')
                MUSIC = True
        elif auto.data == "FSTOP":
            GPIO.output(FIRE,GPIO.LOW)

        rospy.loginfo("auto %s",auto.data)


def callbackT(teleop):
    if teleop.data == "MANUAL CONTROL ENGAGE":
        global SAM
        SAM = False
    if teleop.data == "SAM CONTROL ENGAGE":
        global SAM 
        SAM = True
    if not SAM:
        if teleop.data == "RIGHT":
            GPIO.output(RIGHT,GPIO.HIGH)
        elif teleop.data == "RSTOP":
            GPIO.output(RIGHT,GPIO.LOW)

        if teleop.data == "LEFT":
            GPIO.output(LEFT,GPIO.HIGH)
        elif teleop.data == "LSTOP":
            GPIO.output(LEFT,GPIO.LOW)

    
        if teleop.data == "UP":
            GPIO.output(UP,GPIO.HIGH)
        elif teleop.data == "USTOP":
            GPIO.output(UP,GPIO.LOW)
            

        if teleop.data == "DOWN":
            GPIO.output(DOWN,GPIO.HIGH)
        elif teleop.data == "DSTOP":
            GPIO.output(DOWN,GPIO.LOW)

        
        if teleop.data == "FIRE":
            GPIO.output(FIRE,GPIO.HIGH)
            global MUSIC
            if not MUSIC:
                os.system('mpg321 /media/pi/BE82-0E13/*.mp3 &')
                MUSIC = True
        elif teleop.data == "FSTOP":
            GPIO.output(FIRE,GPIO.LOW)

    rospy.loginfo("teleop %s",teleop.data)


rospy.init_node('controller',anonymous=True)
rospy.Subscriber('teleop',String,callbackT)
rospy.Subscriber('auto',String,callbackA)

rospy.spin()
