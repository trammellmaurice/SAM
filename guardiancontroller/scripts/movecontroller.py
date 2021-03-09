#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String

import RPi.GPIO as GPIO

import os

# pin definitions
GPIO.setmode(GPIO.BOARD)

# set up GPIO
ALL = [3,7,12,18,23]
FIRE,UP,DOWN,RIGHT,LEFT = 3,7,12,18,23
GPIO.setup(ALL,GPIO.OUT,initial=GPIO.LOW)

def callbackA(auto):

    if auto.data == "RIGHT":
        GPIO.output(LEFT,GPIO.LOW)
        GPIO.output(RIGHT,GPIO.HIGH)

    elif auto.data == "LEFT":
        GPIO.output(RIGHT,GPIO.LOW)
        GPIO.output(LEFT,GPIO.HIGH)


    if auto.data == "UP":
        GPIO.output(DOWN,GPIO.LOW)
        GPIO.output(UP,GPIO.HIGH)

    elif auto.data == "DOWN":
        GPIO.output(UP,GPIO.LOW)
        GPIO.output(DOWN,GPIO.HIGH)

    if auto.data == "FIRE":
        GPIO.output(FIRE,GPIO.HIGH)
    elif auto.data == "FSTOP":
        GPIO.output(FIRE,GPIO.LOW)

    rospy.loginfo("auto %s",auto.data)


rospy.init_node('movecontroller',anonymous=True)
rospy.Subscriber('autopilot_commands',String,callbackA)

rospy.spin()