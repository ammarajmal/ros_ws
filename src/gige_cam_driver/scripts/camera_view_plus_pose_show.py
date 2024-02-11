#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np

bridge = CvBridge()

def image_callback(msg):
    # Convert ROS message to OpenCV image
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # Perform ArUco marker detection and pose estimation
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
    parameters = cv2.aruco.DetectorParameters_create()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, distortion_coeffs)
        for i in range(len(ids)):
            cv2.aruco.drawAxis(img, camera_matrix, distortion_coeffs, rvecs[i], tvecs[i], 0.1)

    # Display the image with ArUco markers and axes
    cv2.imshow("Camera Output", img)
    # Wait for a key press for 1 millisecond
    key = cv2.waitKey(1)
    # Check if the q key is pressed
    if key == ord('q'):
        # Close the window
        cv2.destroyAllWindows()

def camera_info_callback(msg):
    global camera_matrix, distortion_coeffs
    # Extract camera intrinsic parameters from CameraInfo message
    camera_matrix = np.array(msg.K).reshape(3, 3)
    distortion_coeffs = np.array(msg.D)

def main():
    rospy.init_node('camera_display')
    # Get the camera name from the ROS parameter server
    camera_name = rospy.get_param("~camera_name", "camera_1")
    # Subscribe to the "/<camera_name>/image_raw" topic
    rospy.Subscriber("/" + camera_name + "/image_raw", Image, image_callback)
    # Subscribe to the "/<camera_name>/camera_info" topic to receive camera calibration data
    rospy.Subscriber("/" + camera_name + "/camera_info", CameraInfo, camera_info_callback)
    # Start the main ROS loop
    rospy.spin()

if __name__ == '__main__':
    main()
