#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2 as cv
import time
import yaml

def load_camera_parameters():
    # Load camera parameters from YAML file in the config directory
    with open(rospy.get_param('~camera_info_file'), 'r') as f:
        camera_params = yaml.safe_load(f)
    return camera_params

def main():
    # Initialize ROS node
    rospy.init_node('camera_publisher', anonymous=True)

    # Load camera parameters
    camera_params = load_camera_parameters()

    # Define GStreamer pipeline string
    pipeline = f"v4l2src device=/dev/video0 ! image/jpeg, width={camera_params['image_width']}, height={camera_params['image_height']}, framerate=60/1 ! decodebin ! videoconvert ! appsink"

    # Create VideoCapture object with GStreamer pipeline
    cap = cv.VideoCapture(pipeline, cv.CAP_GSTREAMER)

    # Check if the camera opened successfully
    if not cap.isOpened():
        rospy.logerr("Error: Could not open camera")
        return

    # Initialize variables for FPS calculation
    fps = 0
    start_time = time.time()
    frame_count = 0

    # Initialize ROS publishers
    image_pub = rospy.Publisher('camera/image_raw', Image, queue_size=10)
    camera_info_pub = rospy.Publisher('camera/camera_info', CameraInfo, queue_size=10)
    bridge = CvBridge()

    rate = rospy.Rate(60)  # Adjust the publishing rate as needed

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            break

        # Increment frame count
        frame_count += 1

        # Calculate elapsed time
        elapsed_time = time.time() - start_time

        # Calculate FPS every second
        if elapsed_time >= 1:
            fps = frame_count / elapsed_time
            start_time = time.time()
            frame_count = 0

        # Display FPS on the frame
        cv.putText(frame, f"FPS: {fps:.2f}", (10, 30), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Convert frame to ROS image message
        ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")

        # Publish ROS image message
        image_pub.publish(ros_image)

        # Publish camera info
        camera_info_msg = CameraInfo()
        camera_info_msg.header.stamp = rospy.Time.now()
        camera_info_msg.width = camera_params['image_width']
        camera_info_msg.height = camera_params['image_height']
        # Set other camera info parameters as needed
        camera_info_pub.publish(camera_info_msg)

        rate.sleep()

    # Release the video capture object
    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
