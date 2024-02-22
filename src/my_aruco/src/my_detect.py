#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np
import cv2.aruco as aruco

class ArucoDetector:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('aruco_detector', anonymous=True)

        # Initialize ROS subscriber for camera images
        rospy.Subscriber('camera/image_raw', Image, self.image_callback)

        # Initialize ArUco detector
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
        self.parameters = aruco.DetectorParameters_create()

        # Initialize ROS publisher for detected markers
        self.marker_pub = rospy.Publisher('camera/markers', Image, queue_size=10)
        self.bridge = CvBridge()

    def image_callback(self, ros_image):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")

        # Detect ArUco markers
        gray = cv.cvtColor(cv_image, cv.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        # Draw detected markers
        aruco.drawDetectedMarkers(cv_image, corners, ids)

        # Publish the image with detected markers
        self.marker_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8"))

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = ArucoDetector()
        detector.run()
    except rospy.ROSInterruptException:
        pass
