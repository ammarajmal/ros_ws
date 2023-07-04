#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
import random

rospy.init_node('example_publisher')
pub = rospy.Publisher('random_numbers', Int32, queue_size=10)

rate = rospy.Rate(1)  # 1 Hz

while not rospy.is_shutdown():
    random_number = random.randint(1, 100)
    pub.publish(random_number)
    rospy.loginfo("Published: %d", random_number)
    rate.sleep()
