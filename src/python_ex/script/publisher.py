#!/usr/bin/env python3
import rospy 
from std_msgs.msg import String

def publisher_node():
    rospy.init_node('py_publisher_node', anonymous=True)
    pub = rospy.Publisher('python_topic', String, queue_size=10)
    rate = rospy.Rate(100) # 100 Hz
    
    while not rospy.is_shutdown():
        message = 'Hello ROS!'
        pub.publish(message)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass