#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

def tx():
    pub = rospy.Publisher('auto',String,queue_size=5)
    rospy.init_node('tx',anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        message = "message {}".format(rospy.get_time())
        rospy.loginfo(message)
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        tx()
    except rospy.ROSInterruptException:
        pass
