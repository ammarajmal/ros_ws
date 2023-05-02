#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

def callback(msg):
    rospy.loginfo("Received {:.3f}".format(msg.data))


if __name__ == '__main__':
    rospy.init_node('sine_subscriber')
    sub = rospy.Subscriber('sine', Float32, callback)
    rospy.spin()