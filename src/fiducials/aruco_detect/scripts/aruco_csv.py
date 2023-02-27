#!/usr/bin/env python3

import rospy
import csv
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Vector3

def callback(data):
    with open('fiducial_transforms.csv', mode='a') as csv_file:
        fieldnames = ['x', 'y', 'z']
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

        writer.writeheader()

        for transform in data.transforms:
            writer.writerow({'x': float(transform.transform.translation.x),
                             'y': float(transform.transform.translation.y),
                             'z': float(transform.transform.translation.z)})
if __name__ == '__main__':
    rospy.init_node('fiducial_transforms_csv', anonymous=True)
    rospy.Subscriber('/camera_1_aruco_detect/fiducial_transforms', FiducialTransformArray, callback)

    rospy.spin()
