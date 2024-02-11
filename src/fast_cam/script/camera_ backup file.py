#!/usr/bin/env python3
# coding=utf-8
import rospy
import mvsdk
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from camera_info_manager import CameraInfoManager
from fast_cam.msg import CameraSpecs as CameraParameters
from fast_cam.srv import SetGain, SetGainResponse
from fast_cam.srv import GetCameraProperties, GetCameraPropertiesResponse
# from std_srvs.srv import Trigger, TriggerResponse
# import sys


class CameraNode:
    """Camera node class"""

    def __init__(self):
        """ Initialize the camera node """
        rospy.init_node('camera_node', anonymous=False)
        # publishers
        self.image_publisher = rospy.Publisher(
            "~image_raw", Image, queue_size=10)
        self.camera_info_publisher = rospy.Publisher(
            "~camera_info", CameraInfo, queue_size=10)
        self.camera_parameters_publisher = rospy.Publisher(
            "~camera_specifications", CameraParameters, queue_size=10)
        self.set_gain_service = rospy.Service(
            "~set_gain", SetGain, self.handle_set_gain)
        self.get_camera_properties_service = rospy.Service(
            "~get_camera_properties", GetCameraProperties, self.handle_get_camera_properties)
        # Camera parameters
        self.camera_parameters_file = "/home/ammar/ros_ws/src/fast_cam/config/camera_info_nuc1.yaml"
        self.camera_name = rospy.get_param("~camera_name", "Camera_0")
        self.default_fps = 100
        self.rate = rospy.Rate(rospy.get_param(
            '~frame_rate', self.default_fps))
        self.device_id = rospy.get_param("~device_id", -1)
        self.device_ip = rospy.get_param("~device_ip", "192.168.1.200")
        self.calibration_file = rospy.get_param(
            "~calibration_file", self.camera_parameters_file)
        self.camera_manager = rospy.get_param("~camera_manager", "opencv")
        self.camera_info_manager = CameraInfoManager(
            cname=self.camera_manager, url=f'file://{self.calibration_file}', namespace=self.camera_manager)

        # self.shutdown_service = rospy.Service("~shutdown", Trigger, self.handle_shutdown)
        self.bridge = CvBridge()
        self.cap = None
        self.gain = 100
        self.camera_frate = 0

        self.camera = None
        self.working_camera = 0

        self.camera_parameters = CameraParameters()

        self.initialize_camera()

    def initialize_camera(self):
        """Initialize and configure the camera"""
        device_list = mvsdk.CameraEnumerateDevice()
        num_devices = len(device_list)
        if num_devices < 1:
            rospy.logerr("Shutting down, No Camera found.")
            rospy.signal_shutdown("Shutting down, No Camera found.")
            return
        rospy.loginfo(f"Found {num_devices} connected camera(s).")
        my_camera = None
        sel_camera = 0
        for cur_device in device_list:
            # print(f'Iterating over device: {cur_device}')
            camera_ip_found = cur_device.acPortType.decode(
                "utf-8").split("-")[0] == self.device_ip
            if camera_ip_found:
                my_camera = cur_device
                break

        if my_camera is None:
            rospy.logerr(f"Camera with IP {self.device_ip} is not found.")
            rospy.signal_shutdown("Camera at specified IP not found.")
        else:
            # Print camera information
            # self.print_camera_info(my_camera)
            try:
                sel_camera = mvsdk.CameraInit(my_camera, -1, -1)

            except mvsdk.CameraException as e:
                rospy.logerr(f"CameraInit Failed: {e.message}")
                rospy.signal_shutdown("CameraInit Failed.")
                return
            # Get camera capability description
            cap = mvsdk.CameraGetCapability(sel_camera)

            # Set output format
            mono_camera = (cap.sIspCapacity.bMonoSensor != 0)
            if mono_camera:
                mvsdk.CameraSetIspOutFormat(
                    sel_camera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
            else:
                mvsdk.CameraSetIspOutFormat(
                    sel_camera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
            # Allocate the RGB buffer
            frame_buffer_size = cap.sResolutionRange.iWidthMax * \
                cap.sResolutionRange.iHeightMax * (1 if mono_camera else 3)

            # rospy.loginfo(f"Resolution: {cap.sResolutionRange.iWidthMax} x {cap.sResolutionRange.iHeightMax}")
            frame_buffer = mvsdk.CameraAlignMalloc(frame_buffer_size, 16)

            mvsdk.CameraSetTriggerMode(sel_camera, 0)
            mvsdk.CameraSetAeState(sel_camera, 0)
            # mvsdk.CameraSetAnalogGain(sel_camera, self.gain)  # 0dB
            mvsdk.CameraSetExposureTime(sel_camera, (1000/self.default_fps)*1000)  # 2ms
            # mvsdk.CameraSetImageResolution(sel_camera, 640, 480)
            mvsdk.CameraPlay(sel_camera)
            self.camera = sel_camera
            self.cap = cap
            self.frame_buffer = frame_buffer

            # rospy.loginfo(f'Camera Resolution: {mvsdk.CameraGetImageResolution(sel_camera).acDescription.decode("utf-8")}')
            # rospy.loginfo(f'Camera Exposure Time: {mvsdk.CameraGetExposureTime(sel_camera)/1000} ms')
            # rospy.loginfo(f'Camera Gain: {mvsdk.CameraGetAnalogGain(sel_camera)}')

            self.camera_parameters.name = self.camera_name
            self.camera_parameters.model = my_camera.acFriendlyName.decode(
                'utf-8')
            self.camera_parameters.serial_number = my_camera.acSn.decode(
                'utf-8')
            self.camera_parameters.ip_address = my_camera.acPortType.decode(
                'utf-8').split('-')[0]

            self.camera_parameters.resolution = mvsdk.CameraGetImageResolution(
                sel_camera).acDescription.decode("utf-8")

            # self.camera_parameters.frame_rate = (self.camera_frate)
            self.camera_parameters.exposure_time = mvsdk.CameraGetExposureTime(
                sel_camera)/1000
            self.camera_parameters.gain = str(
                mvsdk.CameraGetAnalogGain(sel_camera))

            self.camera_info_manager.loadCameraInfo()

    def main_loop(self):
        """Main loop"""
        try:
            while not rospy.is_shutdown():
                frame = self.capture_frame()
                if frame is not None:
                    self.publish_frame(frame)
                else:
                    self.cleanup()
                    rospy.logerr('1. No frame found, leaving the main loop.. ')
                    rospy.signal_shutdown("No frame found.")
                    break
        except Exception as e:
            rospy.logerr(f'2. No frame found, leaving the main loop.. {e}')
            self.cleanup()
            raise

    def capture_frame(self):
        """Capture a frame from the camera"""
        try:
            p_raw_data, frame_head = mvsdk.CameraGetImageBuffer(
                self.camera, 1000)
            mvsdk.CameraImageProcess(
                self.camera, p_raw_data, self.frame_buffer, frame_head)
            if p_raw_data is not None:
                mvsdk.CameraReleaseImageBuffer(self.camera, p_raw_data)
            frame_data = (mvsdk.c_ubyte *
                          frame_head.uBytes).from_address(self.frame_buffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((frame_head.iHeight, frame_head.iWidth,
                                  1 if frame_head.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
            return frame
        except Exception as e:
            rospy.logerr(f"Frame capture failed: {e}")
            return None

    def publish_frame(self, frame):
        """Publish the frame as a ROS Image message"""
        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        ros_image.header.stamp = rospy.Time.now()
        ros_image.header.frame_id = self.camera_name

        camera_info = self.camera_info_manager.getCameraInfo()
        camera_info.header.stamp = ros_image.header.stamp
        camera_info.header.frame_id = ros_image.header.frame_id

        self.image_publisher.publish(ros_image)
        self.camera_info_publisher.publish(camera_info)
        # self.camera_parameters_publisher.publish(self.camera_parameters)
        self.rate.sleep()

    def cleanup(self):
        """Cleanup resources"""
        if self.camera:
            mvsdk.CameraUnInit(self.camera)
            mvsdk.CameraAlignFree(self.frame_buffer)

    def print_camera_info(self, camera):
        """ Print camera information"""
        print("Camera information: ")
        print(f"    Name: {self.camera_name}")
        print(f"    Type: {camera.acProductSeries.decode('utf-8')}")
        print(f"    Model: {camera.acFriendlyName.decode('utf-8')}")
        print(f"    Serial number: {camera.acSn.decode('utf-8')}")
        print(f"    IP: {camera.acPortType.decode('utf-8').split('-')[0]}")
        # print(f"    Resolution: {self.cap.sResolutionRange.iWidthMax} x {self.cap.sResolutionRange.iHeightMax}")
        # print(f"    Port: {camera.acPortType.decode('utf-8').split('-')[1]}")

    def handle_set_gain(self, request):
        """Handle set gain service"""
        try:
            new_gain = request.gain
            print(f"Setting gain to {new_gain}")
            try:
                new_gain = int(new_gain)
            except ValueError:
                rospy.logerr(f"Invalid gain value: {new_gain}")
                return SetGainResponse(False, f"Invalid gain value: {new_gain}")
            mvsdk.CameraSetAnalogGain(self.camera, new_gain)
            self.camera_parameters.gain = str(
                mvsdk.CameraGetAnalogGain(self.camera))
            rospy.loginfo(f"Gain set to {self.camera_parameters.gain}")
            self.gain = new_gain
            return SetGainResponse(True, f"Gain set to {new_gain}")
        except mvsdk.CameraException as e:
            rospy.logerr(f"Failed to set gain: {e.message}")
            return SetGainResponse(False, f"Failed to set gain: {e.message}")
    
    def handle_get_camera_properties(self, request):
        """Handle get camera properties service"""
        try:
            response = GetCameraPropertiesResponse()
            response.name = self.camera_parameters.name
            response.model = self.camera_parameters.model
            response.serial_number = self.camera_parameters.serial_number
            response.ip_address = self.camera_parameters.ip_address
            response.resolution = self.camera_parameters.resolution
            # response.target_fps = self.target_frame_rate
            # response.exposure_time = self.camera_parameters.exposure_time
            # response.gain = self.camera_parameters.gain
            return response
        except mvsdk.CameraException as e:
            rospy.logerr(f"Failed to get camera properties: {e.message}")
            return GetCameraPropertiesResponse(False, f"Failed to get camera properties: {e.message}")

if __name__ == '__main__':
    try:
        camera_node = CameraNode()
        camera_node.main_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        camera_node.cleanup()
