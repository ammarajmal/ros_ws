#!/usr/bin/env python3
import rospy
import numpy as np
from camera_info_manager import CameraInfoManager
from geometry_msgs.msg import PoseStamped
from gige_cam_driver import (CameraInit, CameraException, CameraGetCapability, CameraImageProcess,
                              CAMERA_MEDIA_TYPE_MONO8, CameraSetIspOutFormat, CameraReleaseImageBuffer,
                              CAMERA_MEDIA_TYPE_BGR8, CameraAlignMalloc, CameraGetImageBuffer, c_ubyte,
                              CameraSetTriggerMode, CameraSetAeState, CameraAlignFree, CAMERA_STATUS_TIME_OUT,
                              CameraSetExposureTime, CameraPlay, CameraUnInit, CameraEnumerateDevice)
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import math

bridge = CvBridge()


class Camera(object):
    def __init__(self):
        self.default_fps = 150
        self.DevInfo = None
        self.hCamera = 0
        self.cap = None
        self.pFrameBuffer = 0
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters_create()

        rospy.init_node('camera_node', anonymous=False)
        self.image_publisher = rospy.Publisher('~image_raw', Image, queue_size=10)
        self.camera_info_publisher = rospy.Publisher('~camera_info', CameraInfo, queue_size=10)
        self.aruco_pose_publisher = rospy.Publisher('~aruco_pose', PoseStamped, queue_size=10)

        self.dev_id = rospy.get_param("~device_id", 0)
        self.calibration_file = rospy.get_param("~calibration_file", 0)
        self.camera_manager = rospy.get_param("~camera_manager", 0)
        self.camera_info_manager = CameraInfoManager(
            cname=self.camera_manager, url=f'file://{self.calibration_file}', namespace=self.camera_manager)

        self.camera_info_manager.loadCameraInfo()

    def open(self):
        if self.hCamera > 0:
            return True
        hCamera = 0
        try:
            hCamera = CameraInit(self.DevInfo, -1, -1)
        except CameraException as e:
            print(f"CameraInit Failed({e.error_code}): {e.message}")
            return False
        cap = CameraGetCapability(hCamera)
        monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
        if monoCamera:
            CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8)
        else:
            CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8)
        FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (
            1 if monoCamera else 3)
        pFrameBuffer = CameraAlignMalloc(FrameBufferSize, 16)
        CameraSetTriggerMode(hCamera, 0)
        CameraSetAeState(hCamera, 0)
        CameraSetExposureTime(hCamera, (1000/self.default_fps) * 1000)
        CameraPlay(hCamera)
        self.hCamera = hCamera
        self.pFrameBuffer = pFrameBuffer
        self.cap = cap
        return True

    def close(self):
        if self.hCamera > 0:
            CameraUnInit(self.hCamera)
            self.hCamera = 0
        CameraAlignFree(self.pFrameBuffer)
        self.pFrameBuffer = 0

    def grab(self):
        hCamera = self.hCamera
        pFrameBuffer = self.pFrameBuffer
        try:
            pRawData, FrameHead = CameraGetImageBuffer(hCamera, 200)
            CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            CameraReleaseImageBuffer(hCamera, pRawData)
            frame_data = (c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType ==
                                   CAMERA_MEDIA_TYPE_MONO8 else 3))
            return frame
        except CameraException as e:
            if e.error_code != CAMERA_STATUS_TIME_OUT:
                print(f"CameraGetImageBuffer failed({e.error_code}): {e.message}")
            return None

    def run(self):
        DevList = CameraEnumerateDevice()
        nDev = len(DevList)
        if nDev < 1:
            print("No camera was found!")
            return
        for i, DevInfo in enumerate(DevList):
            print(f"{i}: {DevInfo.GetFriendlyName()} {DevInfo.GetPortType()}")
        self.DevInfo = DevList[self.dev_id]
        if not self.open():
            print("camera not opened")
            exit(0)
        rate = rospy.Rate(150)  # (frames per second)
        rospy.loginfo(f"Camera {self.dev_id} initialized successfully")
        while not rospy.is_shutdown():
            frame = self.grab()
            if frame is not None:
                frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)
                camera_info = self.camera_info_manager.getCameraInfo()
                camera_info.header.stamp = rospy.Time.now()
                camera_info.header.frame_id = self.camera_manager
                camera_matrix = np.array(camera_info.K).reshape((3, 3))
                distortion_coeffs = np.array(camera_info.D)
                # print(type(self.camera_info_manager))
                # print(camera_matrix)
                # print(distortion_coeffs)
                # break
                self.camera_info_publisher.publish(camera_info)

                # Detect ArUco markers and estimate pose
                corners, ids, _ = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.aruco_params)
                if ids is not None:
                    # rospy.loginfo(f"Detected {len(ids)} ArUco markers")
                    # Load camera calibration parameters
                    # camera_matrix = np.array([[3847.274081300875, 0.0, 359.5],
                    #                         [0.0, 3847.274081300875, 269.5],
                    #                         [0.0, 0.0, 1.0]])
                    # distortion_coeffs = np.array([[-1.2010827160226731, -11.237694519054694,
                    #                         0.023040313718269534, 0.016267777509383553, 0.0]])

                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.02, camera_matrix, distortion_coeffs)


                    for i in range(len(ids)):
                        aruco_id = ids[i]
                        rvec = rvecs[i]
                        tvec = tvecs[i]

                        if tvec.size > 0:  # Check if tvec is not empty
                            # Convert rotation vector to quaternion
                            rvec = np.squeeze(rvec)
                            rotation_matrix = cv2.Rodrigues(rvec)[0]
                            w, _ = cv2.Rodrigues(rotation_matrix)
                            qx, qy, qz, qw = self.rotation_matrix_to_quaternion(rotation_matrix)



                            # # Print ArUco marker ID and pose
                            # print(f"Detected ArUco marker {aruco_id}:")
                            # # print(f"Position: ({tvec[0]}, {tvec[1]}, {tvec[2]})")
                            # print(f"Quaternion: ({qx}, {qy}, {qz}, {qw})")
                            # print(f'rvec: {rvec}')
                            # print(f'tvec: {tvec}')
                            # break

                            # Publish the pose
                            pose_msg = PoseStamped()
                            pose_msg.header.stamp = rospy.Time.now()
                            pose_msg.header.frame_id = self.camera_manager
                            pose_msg.pose.position.x = tvec[0][0]
                            pose_msg.pose.position.y = tvec[0][1]
                            pose_msg.pose.position.z = tvec[0][2]
                            pose_msg.pose.orientation.x = qx
                            pose_msg.pose.orientation.y = qy
                            pose_msg.pose.orientation.z = qz
                            pose_msg.pose.orientation.w = qw
                            self.aruco_pose_publisher.publish(pose_msg)
                        else:
                            rospy.logwarn("Empty translation vector (tvec) for ArUco marker detection.")

            # Check if the frame is a valid NumPy array
            if not isinstance(frame, np.ndarray):
                rospy.logerr(f"Error: Invalid frame type: {type(frame)}")
                break
            msg = bridge.cv2_to_imgmsg(frame, "bgr8")
            msg.header.frame_id = self.camera_manager
            msg.header.stamp = rospy.Time.now()
            self.image_publisher.publish(msg)
            rate.sleep()
            if rospy.is_shutdown():
                self.close()

    def rotation_matrix_to_quaternion(self, R):
        """Convert a rotation matrix to a quaternion.

        Args:
            R (numpy.ndarray): 3x3 rotation matrix.

        Returns:
            tuple: Tuple containing x, y, z, w components of the quaternion.
        """
        q = np.zeros(4)
        q[0] = math.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2.0
        q[1] = (R[2, 1] - R[1, 2]) / (4.0 * q[0])
        q[2] = (R[0, 2] - R[2, 0]) / (4.0 * q[0])
        q[3] = (R[1, 0] - R[0, 1]) / (4.0 * q[0])
        return q


import contextlib
if __name__ == '__main__':
    with contextlib.suppress(rospy.ROSInterruptException):
        driver = Camera()
        driver.run()
