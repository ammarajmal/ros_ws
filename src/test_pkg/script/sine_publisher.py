#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

import math

RATE = 100 # Hz (Publishing rate = 10 times per second)
FREQ = 1 # Hz (Frequency of sine wave = 1 Hz)
if __name__ == '__main__':
    rospy.init_node('sine_publisher')
    pub = rospy.Publisher('sine', Float32, queue_size=10)
    rate = rospy.Rate(RATE)
    step = 0
    while not rospy.is_shutdown():
        t = rospy.get_time()
        sine = math.sin(2*math.pi*FREQ*(step/RATE))
        # Log and Publish data
        rospy.loginfo("Publishing {:.3f}".format(sine))
        pub.publish(sine)
        step += 1
        # wait to match the rate
        rate.sleep()