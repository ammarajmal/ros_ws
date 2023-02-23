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

        self.pose_pub = rospy.Publisher('/aruco_pose_cam1', FiducialTransform, queue_size=10)

        self.image_sub = rospy.Subscriber('/camera_1/image_raw', Image, self.image_callback)
        self.camera_info_sub = rospy.Subscriber('/camera_1/camera_info', CameraInfo, self.camera_info_callback)

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

                #  convert rvect to Rodrigues
                rmat, _ = cv2.Rodrigues(rvecs[i][0])
                
                R, _ = cv2.Rodrigues(rvecs[i][0])

                C = -R.T * tvecs
                print(C)
                # position = -R^-1 * tvecs[i][0]
                print('\n\n')
                # print(tvecs[i][0])
                
                # Find the inverse of the rotation matrix
                rmat_inv = np.linalg.inv(rmat)
                #  Find the coordinates of the camera in the world frame

                cam_pos = -np.matmul(rmat_inv, tvecs[i].T)
                #  Find the rotation matrix of the camera in the world frame
                cam_rot = rmat_inv.T    #  Transpose of the inverse of the rotation matrix

                pose_fin = FiducialTransform()
                pose_fin.fiducial_id = ids[i][0]
                pose_fin.transform.translation.x = cam_pos[0][0]
                pose_fin.transform.translation.y = cam_pos[1][0]
                pose_fin.transform.translation.z = cam_pos[2][0]
                pose_fin.transform.rotation.x = cam_rot[0][0]
                pose_fin.transform.rotation.y = cam_rot[1][0]
                pose_fin.transform.rotation.z = cam_rot[2][0]
       
       
                # print('x:',  cam_pos[0][0], 'y:', cam_pos[1][0], 'z:', cam_pos[2][0])
                # x = cam_pos[0][0]
                # y = cam_pos[1][0]
                # z = cam_pos[2][0]
                # # save x, y, z to a csv file in individual columns
                # with open('cam_pos_fin.csv', 'a') as f:
                #     f.write(str(x) + ',' + str(y) + ',' + str(z) + '\n')   


                self.pose_pub.publish(pose_fin)

if __name__ == '__main__':
    rospy.init_node('aruco_pose_cam1_estimator')
    node = ArucoPoseEstimator()
    rospy.spin()
