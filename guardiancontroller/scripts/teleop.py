#!/usr/bin/env python
from inputs import get_gamepad


import time
import sys

import rospy
from std_msgs.msg import String

ARMED = False
SAM = True

pub = rospy.Publisher('teleop_commands',String,queue_size=1)
rospy.init_node('gpad',anonymous=True)
rate= rospy.Rate(10)

print("SAM CONTROL")

while not rospy.is_shutdown():
    events = get_gamepad()

    for event in events:
        if event.code == "BTN_TL2" and event.state == 1:
            SAM = False
            rospy.loginfo("MANUAL CONTROL ENGAGE")
            pub.publish("MANUAL CONTROL ENGAGE")
            rate.sleep()
        elif event.code == "BTN_TL2" and event.state == 0:
            SAM = True
            ARMED = False
            rospy.loginfo("SAM CONTROL ENGAGE")
            pub.publish("SAM CONTROL ENGAGE")
            rate.sleep()
        if not SAM:
            if event.code == "BTN_WEST" and event.state == 1:
                if not ARMED:
                    ARMED = True
                    rospy.loginfo("ARMED")
                    pub.publish("ARMED")
                    rate.sleep()
                else:
                    ARMED = False
                    rospy.loginfo("DISARMED")
                    pub.publish("DISARMED")
                    rate.sleep()


            elif event.code == "BTN_DPAD_RIGHT" and event.state == 1:
                rospy.loginfo("RIGHT")
                pub.publish("RIGHT")
                rate.sleep()
            elif event.code == "BTN_DPAD_RIGHT" and event.state == 0:
                pub.publish("RSTOP")
                rate.sleep()

            elif event.code == "BTN_DPAD_LEFT" and event.state == 1:

                rospy.loginfo("LEFT")
                pub.publish("LEFT")
                rate.sleep()
            elif event.code == "BTN_DPAD_LEFT" and event.state == 0:
                pub.publish("LSTOP")



            elif event.code == "BTN_DPAD_UP" and event.state == 1:
                rospy.loginfo("UP")
                pub.publish("UP")
                rate.sleep()
            elif event.code == "BTN_DPAD_UP" and event.state == 0:
                pub.publish("USTOP")


            elif event.code == "BTN_DPAD_DOWN" and event.state == 1:
                rospy.loginfo("DOWN")
                pub.publish("DOWN")
                rate.sleep()
            elif event.code == "BTN_DPAD_DOWN" and event.state == 0:
                pub.publish("DSTOP")


            elif event.code == "BTN_TR" and event.state == 1:
                if ARMED:
                    rospy.loginfo("FIRE")
                    pub.publish("FIRE")
                    rate.sleep()
                else:
                    rospy.loginfo("MUST ARM")
                    pub.publish("MUST ARM")
                    rate.sleep()

            elif event.code == "BTN_TR" and event.state == 0:
                pub.publish("FSTOP")
