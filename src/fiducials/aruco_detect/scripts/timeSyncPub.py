#!/usr/bin/env python

import rospy
import message_filters
from fiducial_msgs.msg import FiducialTransformArray

# Global publishers
pub_nuc2_sync = None
pub_nuc3_sync = None

def callback(nuc2_msg, nuc3_msg):
    nuc2_time = nuc2_msg.header.stamp
    nuc3_time = nuc3_msg.header.stamp
    time_diff = abs(nuc2_time.to_sec() - nuc3_time.to_sec())
    rospy.loginfo("Time difference between messages: {} seconds".format(time_diff))

    # Publish the synchronized messages
    pub_nuc2_sync.publish(nuc2_msg)
    pub_nuc3_sync.publish(nuc3_msg)

def main():
    global pub_nuc2_sync, pub_nuc3_sync

    rospy.init_node('synchronization_node', anonymous=True)

    nuc2_sub = message_filters.Subscriber('/nuc2/fiducial_transforms', FiducialTransformArray)
    nuc3_sub = message_filters.Subscriber('/nuc3/fiducial_transforms', FiducialTransformArray)

    ats = message_filters.ApproximateTimeSynchronizer([nuc2_sub, nuc3_sub], 10, 0.1)
    ats.registerCallback(callback)

    # Initialize publishers
    pub_nuc2_sync = rospy.Publisher('/nuc2/fiducial_transforms_sync', FiducialTransformArray, queue_size=10)
    pub_nuc3_sync = rospy.Publisher('/nuc3/fiducial_transforms_sync', FiducialTransformArray, queue_size=10)

    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
