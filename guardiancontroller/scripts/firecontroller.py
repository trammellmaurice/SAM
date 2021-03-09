#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import String

import RPi.GPIO as GPIO

import os

# pin definitions
GPIO.setmode(GPIO.BOARD)

# set up GPIO
FIRE = 3
GPIO.setup(FIRE,GPIO.OUT,initial=GPIO.LOW)

def callbackA(auto):

    if auto.data == "FIRE":
        GPIO.output(FIRE,GPIO.HIGH)
        global rate
        rate.sleep()
        GPIO.output(FIRE,GPIO.LOW)

    rospy.loginfo("auto %s",auto.data)


rospy.init_node('firecontroller',anonymous=True)
rate = rospy.Rate(0.5)
rospy.Subscriber('autopilot_commands',String,callbackA)
rospy.spin()
