#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge
import numpy as np
from fiducial_msgs.msg import FiducialTransform


from sensor_msgs.msg import CameraInfo, Image

class ArucoPoseEstimator:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.pose_pub = rospy.Publisher('/aruco_pose_cam2', FiducialTransform, queue_size=10)

        self.image_sub = rospy.Subscriber('/camera_2/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera_2/camera_info', CameraInfo, self.camera_info_callback)

        self.camera_matrix = None
        self.dist_coeffs = None

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape((3, 3))
        self.dist_coeffs = np.array(msg.D)

    def image_callback(self, msg):
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
        parameters =  cv2.aruco.DetectorParameters_create()

        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:

            rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, 0.053, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                # cv2.aruco.drawAxis(image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.1)
                # rospy.loginfo('Found aruco markers: %s', ids[i])

                pose = FiducialTransform()
                pose.fiducial_id = ids[i][0]
                pose.transform.translation.x = tvecs[i][0][0]
                pose.transform.translation.y = tvecs[i][0][1]
                pose.transform.translation.z = tvecs[i][0][2]
                pose.transform.rotation.x = rvecs[i][0][0]
                pose.transform.rotation.y = rvecs[i][0][1]
                pose.transform.rotation.z = rvecs[i][0][2]
                self.pose_pub.publish(pose)
                Rodrigues = cv2.Rodrigues(rvecs[i])

if __name__ == '__main__':
    rospy.init_node('aruco_pose_cam2_estimator')
    node = ArucoPoseEstimator()
    rospy.spin()
