#!/usr/bin/env python

import time
import subprocess
import threading 

import rospy
from std_msgs.msg import String

STATES = {'help': ['mpg321', '-l 0','/media/pi/BE82-0E13/sad.mp3'],
'attack': ['mpg321','-l 0', '/media/pi/BE82-0E13/attack.mp3'],
'idle': ['mpg321', '-l 0', '/media/pi/BE82-0E13/idle.mp3']}

last = None
process = None


def callback(state):
    if state.data == "a" and last != 'a':
        if (last == 'h' or last == 'i'):
            global process
            process.kill()
        global STATES
        global process
        process = subprocess.Popen(STATES['attack'])


    elif state.data == "i" and last != 'i':
        if (last == 'h' or last == 'a'):
            global process
            process.kill()
        global STATES
        global process
        process = subprocess.Popen(STATES['idle'])

    elif state.data == "h" and last != 'h':
        if (last == 'i' or last == 'a'):
            global process
            process.kill()
        global STATES
        global process
        process = subprocess.Popen(STATES['help'])

    global last
    last = state.data

rospy.init_node('communicator',anonymous=True)
threading.Thread(target=running).start()
rospy.Subscriber('status',String,callback)
rospy.spin()
