#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class MarkerDisplay:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('marker_display', anonymous=True)

        # Initialize ROS subscriber for detected markers
        rospy.Subscriber('camera/markers', Image, self.marker_callback)

        # Initialize OpenCV window
        cv2.namedWindow("Detected Markers", cv2.WINDOW_NORMAL)

        # Initialize CvBridge
        self.bridge = CvBridge()

    def marker_callback(self, ros_image):
        # Convert ROS image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, desired_encoding="bgr8")

        # Display the image with detected markers
        cv2.imshow("Detected Markers", cv_image)
        cv2.waitKey(1)

    def run(self):
        # Keep the node running
        rospy.spin()

if __name__ == '__main__':
    try:
        marker_display = MarkerDisplay()
        marker_display.run()
    except rospy.ROSInterruptException:
        pass
