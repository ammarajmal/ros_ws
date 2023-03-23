#!/usr/bin/env python3
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

def image_callback(msg):
    # Convert ROS message to OpenCV image
    img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    # Display the image
     # Display the image in a window called "Camera Output" with size 640x480
    cv2.namedWindow("Camera Output", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Camera Output", 640, 480)
    cv2.imshow("Camera Output", img)
    # Wait for a key press for 1 millisecond
    key = cv2.waitKey(1)
    # Check if the q key is pressed
    if key == ord('q'):
        # Close the window
        cv2.destroyAllWindows()

def main():
    rospy.init_node('camera_display')
    # Get the camera name from the ROS parameter server
    camera_name = rospy.get_param("~camera_name", "camera_1")
    # Subscribe to the "/<camera_name>/image_raw" topic
    rospy.Subscriber("/" + camera_name + "/image_raw", Image, image_callback)
    # Start the main ROS loop
    rospy.spin()

if __name__ == '__main__':
    main()
