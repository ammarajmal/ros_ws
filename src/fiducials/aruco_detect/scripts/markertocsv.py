#!/usr/bin/env python3
import rospy
import csv
# from fiducial_msgs.msg import FiducialTransforms
from fiducial_msgs.msg import FiducialTransformArray


def callback(data):
    # Open the CSV file in write mode
    with open('data.csv', mode='w') as file:
        # Create a CSV writer object
        writer = csv.writer(file)
        # Write the header row
        writer.writerow(['ID', 'X', 'Y', 'Z'])
        # Write the data to the CSV file
        for fiducial in data.transforms:
            writer.writerow([fiducial.fiducial_id,
                             fiducial.transform.translation.x,
                             fiducial.transform.translation.y,
                             fiducial.transform.translation.z])

def listener():
    rospy.init_node('marker_to_csv', anonymous=True)
    rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
