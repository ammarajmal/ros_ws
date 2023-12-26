#!/usr/bin/env python3
# coding=utf-8
import rospy
import cv2
import mvsdk
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from camera_info_manager import CameraInfoManager
# from std_srvs.srv import Trigger, TriggerResponse
import sys
class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node', anonymous=False)
        self.image_pub = rospy.Publisher("~image_raw", Image, queue_size=10)
        self.camera_info_publisher = rospy.Publisher("~camera_info", CameraInfo, queue_size=10)
        
        self.device_id = rospy.get_param("~device_id", 0)
        self.device_ip = rospy.get_param("~device_ip", "192.168.1.101")
        self.calibration_file = rospy.get_param("~calibration_file", "")
        self.camera_manager = rospy.get_param("~camera_manager", "opencv")
        
        # self.shutdown_service = rospy.Service("~shutdown", Trigger, self.handle_shutdown)
        
        self.rate = rospy.Rate(125)
        self.camera = None
        
        self.initialize_camera()
        print('somewhere here')
    def initialize_camera(self):
        # Initialize and configure the camera
        device_list = mvsdk.CameraEnumerateDevice()
        num_devices = len(device_list)
        if num_devices < 1:
            rospy.logerr("No camera is connected.")
            rospy.signal_shutdown("No Camera found.")
            # return
        rospy.loginfo(f"Found {num_devices} connected camera(s).")
        for cur_device in device_list:
            if cur_device.acPortType.decode("utf-8").split("-")[0] == self.device_ip:
                self.camera = cur_device
                self.print_camera_info(self.camera)
                break
        else:
            rospy.logerr(f"Camera with IP {self.device_ip} is not found.")
            rospy.signal_shutdown("Camera at specified IP not found.")
                # sys.exit()
                # Exit the program cleanly
        # Rest of the code...
    def print_camera_info(self, camera):
        # Print camera information
        print("Camera information: ")
        print(f"    Friendly name: {camera.acFriendlyName.decode('utf-8')}")
        print(f"    IP: {camera.acPortType.decode('utf-8').split('-')[0]}")
        print(f"    Port: {camera.acPortType.decode('utf-8').split('-')[1]}")
        print(f"    Model: {camera.acProductSeries.decode('utf-8')}")
        print(f"    Serial number: {camera.acSn.decode('utf-8')}")
        # rospy.loginfo(f"    Resolution: {camera.sResolutionRange.iWidthMax}x{camera.sResolutionRange.iHeightMax}")
        # rospy.loginfo(f"    Exposure time: {camera.sExposeDesc.uiExposeTimeMin} - {camera.sExposeDesc.uiExposeTimeMax}")
        # rospy.loginfo(f"    Gain: {camera.sExposeDesc.uiAnalogGainMin} - {camera.sExposeDesc.uiAnalogGainMax}")
        # rospy.loginfo(f"    Frame rate: {camera.sFrameRateRange.iFrameRateMin} - {camera.sFrameRateRange.iFrameRateMax}")
        # rospy.loginfo(f"    Pixel format: {camera.sIspCapacityDescription.acPixelFormats.decode('utf-8')}")
        # rospy.loginfo(f"    Binning: {camera.sIspCapacityDescription.bMonoSensor}")
        # rospy.loginfo(f"    Binning: {camera.sIspCapacityDescription.bBin}")

if __name__ == '__main__':
    try:
        camera_node = CameraNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass