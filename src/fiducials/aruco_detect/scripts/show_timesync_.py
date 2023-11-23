#!/usr/bin/env python
import rospy
from fiducial_msgs.msg import FiducialTransformArray

def callback_nuc2(data):
    global last_time_nuc2
    last_time_nuc2 = data.header.stamp

def callback_nuc3(data):
    global last_time_nuc3
    last_time_nuc3 = data.header.stamp

    # Compare timestamps and calculate the time difference
    if last_time_nuc2 and last_time_nuc3:
        time_diff = abs((last_time_nuc2.secs + last_time_nuc2.nsecs * 1e-9) - 
                        (last_time_nuc3.secs + last_time_nuc3.nsecs * 1e-9))
        rospy.loginfo("Time difference (in seconds): {:.6f}".format(time_diff))

def listener():
    rospy.init_node('sync_checker', anonymous=True)

    rospy.Subscriber('/nuc2/fiducial_transforms', FiducialTransformArray, callback_nuc2)
    rospy.Subscriber('/nuc3/fiducial_transforms', FiducialTransformArray, callback_nuc3)

    rospy.spin()

if __name__ == '__main__':
    last_time_nuc2 = None
    last_time_nuc3 = None
    listener()
    
    
    
