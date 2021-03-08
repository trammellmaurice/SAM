#!/usr/bin/env python

import time
import subprocess

import rospy
from std_msgs.msg import String

STATES = {'help': ['mpg321', '-l 0','/media/marsz/BE82-0E13/sad.mp3'],
'attack': ['mpg321','-l 0', '/media/marsz/BE82-0E13/attack.mp3'],
'idle': ['mpg321', '-l 0', '/media/marsz/BE82-0E13/idle.mp3']}

last = None

def callback(state):
    if state == "a" and last != 'a':
        if (last == 'h' or last == 'i'):
            process.kill()
        global STATES
        process = subprocess.Popen(STATES['attack'])

    elif state == "h" and last != 'h':
        if (last == 'a' or last == 'i'):
            process.kill()
        global STATES
        process = subprocess.Popen(STATES['help'])

    elif state == "i" and last != 'i':
        if (last == 'h' or last == 'a'):
            process.kill()
        global STATES
        process = subprocess.Popen(STATES['idle'])

    global last
    last = state

rospy.init_node('communicator',anonymous=True)
rospy.Subscriber('status',String,callback)
rospy.spin()
