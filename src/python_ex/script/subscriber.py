#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo("Received: %s", data.data)
    
def subscriber_node():
    rospy.init_node('py_subscriber_node', anonymous=True)
    rospy.Subscriber('python_topic', String, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber_node()