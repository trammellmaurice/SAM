#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String

import RPi.GPIO as GPIO

import os

# pin definitions
GPIO.setmode(GPIO.BOARD)

# set up GPIO
ALL = [7,12,18,23,32]
UP,DOWN,RIGHT,LEFT,SLOW = 7,12,18,23,32
GPIO.setup(ALL,GPIO.OUT,initial=GPIO.LOW)

SAM = False

def callbackT(tele):
    if tele.data == "MANUAL CONTROL ENGAGE":
        global SAM
        SAM = False

    elif tele.data == "SAM CONTROL ENGAGE":
        global SAM
        SAM = True

    if not SAM:
        if tele.data == "RIGHT":
            GPIO.output(SLOW,GPIO.LOW)
            GPIO.output(LEFT,GPIO.LOW)
            GPIO.output(RIGHT,GPIO.HIGH)

        elif tele.data == "LEFT":
            GPIO.output(SLOW,GPIO.LOW)
            GPIO.output(RIGHT,GPIO.LOW)
            GPIO.output(LEFT,GPIO.HIGH)

        elif tele.data == "XSTOP":
            GPIO.output(RIGHT,GPIO.LOW)
            GPIO.output(LEFT,GPIO.LOW)

        if tele.data == "UP":
            GPIO.output(SLOW,GPIO.LOW)
            GPIO.output(DOWN,GPIO.LOW)
            GPIO.output(UP,GPIO.HIGH)

        elif tele.data == "DOWN":
            GPIO.output(SLOW,GPIO.LOW)
            GPIO.output(UP,GPIO.LOW)
            GPIO.output(DOWN,GPIO.HIGH)

        elif tele.data == "YSTOP":
            GPIO.output(UP,GPIO.LOW)
            GPIO.output(DOWN,GPIO.LOW)

def callbackA(auto):
    global SAM
    if SAM:

        if auto.data == "RIGHT":
            GPIO.output(SLOW,GPIO.LOW)
            GPIO.output(LEFT,GPIO.LOW)
            GPIO.output(RIGHT,GPIO.HIGH)

        elif auto.data == "LEFT":
            GPIO.output(SLOW,GPIO.LOW)
            GPIO.output(RIGHT,GPIO.LOW)
            GPIO.output(LEFT,GPIO.HIGH)

        elif auto.data == "SRIGHT":
            print('slow')
            GPIO.output(LEFT,GPIO.LOW)
            GPIO.output(RIGHT,GPIO.HIGH)
            GPIO.output(SLOW,GPIO.HIGH)

        elif auto.data == "SLEFT":
            GPIO.output(RIGHT,GPIO.LOW)
            GPIO.output(LEFT,GPIO.HIGH)
            GPIO.output(SLOW,GPIO.HIGH)

        elif auto.data == "XSTOP":
            GPIO.output(RIGHT,GPIO.LOW)
            GPIO.output(LEFT,GPIO.LOW)


        if auto.data == "UP":
            GPIO.output(SLOW,GPIO.LOW)
            GPIO.output(DOWN,GPIO.LOW)
            GPIO.output(UP,GPIO.HIGH)

        elif auto.data == "DOWN":
            GPIO.output(SLOW,GPIO.LOW)
            GPIO.output(UP,GPIO.LOW)
            GPIO.output(DOWN,GPIO.HIGH)

        elif auto.data == "SUP":
            GPIO.output(DOWN,GPIO.LOW)
            GPIO.output(UP,GPIO.HIGH)
            GPIO.output(SLOW,GPIO.HIGH)

        elif auto.data == "SDOWN":
            GPIO.output(UP,GPIO.LOW)
            GPIO.output(DOWN,GPIO.HIGH)
            GPIO.output(SLOW,GPIO.HIGH)

        elif auto.data == "YSTOP":
            GPIO.output(UP,GPIO.LOW)
            GPIO.output(DOWN,GPIO.LOW)

    rospy.loginfo("auto %s",auto.data)


rospy.init_node('movecontroller',anonymous=True)
rospy.Subscriber('teleop_commands',String,callbackT)
rospy.Subscriber('autopilot_commands',String,callbackA)

rospy.spin()
