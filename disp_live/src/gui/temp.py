#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#!/usr/bin/env python
import rospy
from fiducial_msgs.msg import FiducialTransformArray

def fiducial_transforms_callback(data):
    for transform in data.transforms:
        fiducial_id = transform.fiducial_id
        if fiducial_id is not None:
            print(f"Fiducial ID Detected: {fiducial_id}")
        else:
            print("Fiducial ID Not Detected")

def listener():
    rospy.init_node('fiducial_transforms_listener', anonymous=True)
    rospy.Subscriber('/nuc1/fiducial_transforms', FiducialTransformArray, fiducial_transforms_callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
