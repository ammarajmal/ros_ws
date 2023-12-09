#!/usr/bin/env python

import rospy
import message_filters
from fiducial_msgs.msg import FiducialTransformArray

def callback(nuc2_msg, nuc3_msg):
    nuc2_time = nuc2_msg.header.stamp
    nuc3_time = nuc3_msg.header.stamp
    time_diff = abs(nuc2_time.to_sec() - nuc3_time.to_sec())
    rospy.loginfo("Time difference between messages: {} seconds".format(time_diff))

def main():
    rospy.init_node('synchronization_node', anonymous=True)

    nuc2_sub = message_filters.Subscriber('/nuc2/fiducial_transforms', FiducialTransformArray)
    nuc3_sub = message_filters.Subscriber('/nuc3/fiducial_transforms', FiducialTransformArray)

    # ApproximateTime synchronizer
    ats = message_filters.ApproximateTimeSynchronizer([nuc2_sub, nuc3_sub], 10, 0.1)
    ats.registerCallback(callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
