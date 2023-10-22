#!/usr/bin/env python3

import rospy
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import Header
from fiducial_msgs.msg import FiducialTransform
from geometry_msgs.msg import Transform, Vector3, Quaternion
import math

def sine_wave_publisher():
    # Initialize the ROS node
    rospy.init_node('sine_wave_publisher', anonymous=True)
    rospy.loginfo("Sine wave publisher node started")
    
    # Get the amplitude and frequency parameters
    amplitude = rospy.get_param('~amplitude', 2.0)
    frequency = rospy.get_param('~frequency', 4.0)
    node_name = rospy.get_param('~node_name', 'nuc1')
    fiducial_identifier = rospy.get_param('~fiducial_identifier', 0)
    rospy.loginfo(f"Amplitude: {amplitude}, Frequency: {frequency}, Node Name: {node_name}, Fiducial ID: {fiducial_identifier}")
    
    # Create a publisher for the `/fiducial_transforms` topic
    pub = rospy.Publisher(f'/{node_name}/fiducial_transforms', FiducialTransformArray, queue_size=10)

    # Create a ROS message
    fiducial_transform_array = FiducialTransformArray()
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "base_link"  # Change the frame_id to match your setup
    fiducial_transform_array.header = header
    fiducial_transform_array.transforms = []

    # Define the parameters of the sine wave
    # amplitude = 2.0
    # frequency = 4.0  # Hz
    rate = rospy.Rate(10)  # Publish at 10 Hz

    # Main loop to publish the sine wave
    while not rospy.is_shutdown():
        # Calculate the x-coordinate based on the sine wave
        current_time = rospy.Time.now()
        x = amplitude * math.sin(2 * math.pi * frequency * current_time.to_sec())

        # Create a FiducialTransform message
        fiducial_transform = FiducialTransform()
        fiducial_transform.fiducial_id = fiducial_identifier  # Set the fiducial ID as needed
        transform = Transform()
        transform.translation = Vector3(x, 0.0, 0.0)
        transform.rotation = Quaternion(0.0, 0.0, 0.0, 1.0)
        fiducial_transform.transform = transform
        fiducial_transform.image_error = 0.0
        fiducial_transform.object_error = 0.0
        fiducial_transform.fiducial_area = 0.0

        # Append the FiducialTransform to the array
        fiducial_transform_array.transforms.append(fiducial_transform)

        # Publish the message
        pub.publish(fiducial_transform_array)

        rate.sleep()

if __name__ == '__main__':
    try:
        sine_wave_publisher()
    except rospy.ROSInterruptException:
        pass
