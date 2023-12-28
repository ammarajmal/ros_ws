#!/usr/bin/env python3
# coding=utf-8

import rospy
import cv2
import numpy as np
import mvsdk
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from camera_info_manager import CameraInfoManager

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        self.image_pub = rospy.Publisher("~image_raw", Image, queue_size=10)
        self.camera_info_publisher = rospy.Publisher("~camera_info", CameraInfo, queue_size=10)
        self.bridge = CvBridge()
        self.camera_info_manager = CameraInfoManager(cname="camera", namespace="camera")
        self.initialize_camera()

    def initialize_camera(self):
        # Initialize and configure the camera
        DevList = mvsdk.CameraEnumerateDevice()
        nDev = len(DevList)
        if nDev < 1:
            rospy.logerr("No camera was found!")
            return

        DevInfo = DevList[0]  # Assuming the first camera is used
        try:
            self.hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
        except mvsdk.CameraException as e:
            rospy.logerr("CameraInit Failed: {}".format(e.message))
            return

        # Get camera capability description
        self.cap = mvsdk.CameraGetCapability(self.hCamera)

        # Set output format
        monoCamera = (self.cap.sIspCapacity.bMonoSensor != 0)
        if monoCamera:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
        else:
            mvsdk.CameraSetIspOutFormat(self.hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

        # Start the SDK internal image acquisition thread
        mvsdk.CameraPlay(self.hCamera)

        # Allocate the RGB buffer
        FrameBufferSize = self.cap.sResolutionRange.iWidthMax * self.cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
        self.pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    def capture_frame(self):
        # Capture a frame from the camera
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.hCamera, 200)
            mvsdk.CameraImageProcess(self.hCamera, pRawData, self.pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(self.hCamera, pRawData)

            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
            return frame
        except mvsdk.CameraException as e:
            rospy.logerr("Frame capture failed: {}".format(e.message))
            return None

    def publish_frame(self, frame):
        # Publish the frame as a ROS Image message
        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        ros_image.header.stamp = rospy.Time.now()
        ros_image.header.frame_id = "camera"

        camera_info = self.camera_info_manager.getCameraInfo()
        camera_info.header.stamp = ros_image.header.stamp
        camera_info.header.frame_id = ros_image.header.frame_id

        self.image_pub.publish(ros_image)
        self.camera_info_publisher.publish(camera_info)

    def main_loop(self):
        while not rospy.is_shutdown():
            frame = self.capture_frame()
            if frame is not None:
                self.publish_frame(frame)
            rospy.sleep(0.1)  # Adjust the sleep time as needed

    def cleanup(self):
        # Cleanup resources
        if self.hCamera:
            mvsdk.CameraUnInit(self.hCamera)
            mvsdk.CameraAlignFree(self.pFrameBuffer)

if __name__ == '__main__':
    camera_node = CameraNode()
    try:
        camera_node.main_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        camera_node.cleanup()
