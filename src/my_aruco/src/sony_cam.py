#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge
import cv2 as cv
import time
import yaml

bridge = CvBridge()
class SonyCamera(object):
    def __init__(self):
        self.cap = cv.VideoCapture(0)
        rospy.init_node('sony_camera', anonymous=False)
        self.image_publisher = rospy.Publisher('~image_raw', Image, queue_size=10)
        self.camera_info_publisher = rospy.Publisher('~camera_info', CameraInfo, queue_size=10)
        self.calibration_file = rospy.get_param("~calibration_file", 0)
        self.camera_manager = rospy.get_param("~camera_manager", 0)
        self.camera_info_manager = CameraInfoManager(cname=self.camera_manager, url=f'file://{self.calibration_file}', namespace=self.camera_manager)
        self.camera_info_manager.loadCameraInfo()
    def open(self):
        # method to open the camera
        if not self.cap.isOpened():
            self.cap = cv.VideoCapture(0)
        return self.cap.isOpened()
    def close(self):
        # method to close the camera
        if self.cap.isOpened():
            self.cap.release()
    def grab(self):
        # method to grab the frames from the camera
        try:
            print('Opening camera from GRAB')
            if not self.open():
                print('Error: Could not open camera')
                return
            fps = 0
            start_time = time.time()
            frame_count = 0
            rate = rospy.Rate(60)
            print('Rate set at 60 FPS')
            while not rospy.is_shutdown():
                ret, frame = self.cap.read()
                print('Reading Frame')
                if ret:
                    print('Frame read')
                    frame = cv.resize(frame, (640,480), interpolation = cv.INTER_LINEAR)

                    ros_image_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                    ros_image_msg.header.frame_id = self.camera_manager
                    ros_image_msg.header.stamp = rospy.Time.now()
                    self.image_publisher.publish(ros_image_msg)
                    camera_info_msg = self.camera_info_manager.getCameraInfo()
                    camera_info_msg.header.frame_id = self.camera_manager
                    camera_info_msg.header.stamp = rospy.Time.now()

                    self.camera_info_publisher.publish(camera_info_msg)
                    rate.sleep()
                    if rospy.is_shutdown():
                        self.close()
        except rospy.ROSInterruptException:
            self.close()

import contextlib
if __name__ == '__main__':
    with contextlib.closing(SonyCamera()) as cam:
        cam.grab()
