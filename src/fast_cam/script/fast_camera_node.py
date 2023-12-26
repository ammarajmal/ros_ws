#!/usr/bin/env python3
# coding=utf-8
import rospy
import cv2
import mvsdk
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from camera_info_manager import CameraInfoManager
class CameraNode:
    def __init__(self):
        rospy.init_node('fast_camera_node', anonymous=False)
        self.image_pub = rospy.Publisher("~image_raw", Image, queue_size=10)
        self.camera_info_publisher = rospy.Publisher("~camera_info", CameraInfo, queue_size=10)
        self.device_id = rospy.get_param("~device_id", 0)
        self.device_ip = rospy.get_param("~device_ip", "192.168.1.101")
        self.calibration_file = rospy.get_param("~calibration_file", "")
        self.camera_manager = rospy.get_param("~camera_manager", "opencv")
        
        
        
        self.missed_frames = 0
        self.bridge = CvBridge()
        self.rate = rospy.Rate(125)
        self.fast_camera = None
        self.frame_buffer = None
        self.camera_info_manager = CameraInfoManager(cname=self.camera_manager,url=f'file://{self.calibration_file}' ,namespace=self.camera_manager)
        self.camera_info_manager.loadCameraInfo()
        self.initialize_camera()
        self.set_snapshot_resolution(640, 480)
        rospy.sleep(1)
        self.get_snapshot_resolution()
        
    def set_snapshot_resolution(self, width, height):
        resolution = mvsdk.tSdkImageResolution()  # Create a resolution structure
        resolution.iWidth = width
        resolution.iHeight = height
        mvsdk.CameraSetResolutionForSnap(self.fast_camera, resolution)


    def get_snapshot_resolution(self):
        if self.fast_camera is not None:
            try:
                resolution = mvsdk.CameraGetResolutionForSnap(self.fast_camera)
                if resolution.iWidth > 0 and resolution.iHeight > 0:
                    rospy.loginfo(f"Current snapshot resolution: Width: {resolution.iWidth}, Height: {resolution.iHeight}")
                else:
                    rospy.logwarn("Snapshot resolution is not set or camera is not responding.!!!")
            except Exception as e:
                rospy.logerr(f"Error getting snapshot resolution: {e}")
        else:
            rospy.logwarn("Camera is not initialized.")




    def initialize_camera(self):
        device_list = mvsdk.CameraEnumerateDevice()
        if not device_list:
            rospy.loginfo("No camera was found!")
            return
        rospy.loginfo(f"Found {len(device_list)} cameras.")
        rospy.loginfo(f"Using camera {self.device_id + 1}/2.")
        
        

        device_info = device_list[0]
        # device_info.
        
        print('------------------')
        print(f'Camera Series: {device_info.acProductSeries.decode("utf-8")}')
        print(f'Camera Name: {device_info.acProductName.decode("utf-8")}')
        print(f'Camera Model: {device_info.acFriendlyName.decode("utf-8")}')
        print(f'Camera Link Name: {device_info.acLinkName.decode("utf-8")}')
        print(f'Camera IP Address: {device_info.acPortType.decode("utf-8").split("-")[0]}')
        print(f'Camera Port Details: {device_info.acPortType.decode("utf-8")}')
        print(f'Camera Serial: {device_info.acSn.decode("utf-8")}')
        print(f'Camera Instance: {device_info.uInstance}')
        camera_ip = device_info.GetPortType()
        rospy.loginfo(camera_ip.split("-")[0])
        rospy.loginfo(f'Camera IP Address: {self.device_ip}')
        print('------------------')
        self.setup_camera(device_info)
        # print(f'Resolution: {self.get_snapshot_resolution()}')
    def setup_camera(self, device_info):
        try:
            self.fast_camera = mvsdk.CameraInit(device_info, -1, -1)
            cap = mvsdk.CameraGetCapability(self.fast_camera)
            camera_format = mvsdk.CAMERA_MEDIA_TYPE_MONO8 if cap.sIspCapacity.bMonoSensor else mvsdk.CAMERA_MEDIA_TYPE_BGR8
            mvsdk.CameraSetIspOutFormat(self.fast_camera, camera_format)
            mvsdk.CameraSetTriggerMode(self.fast_camera, 0)
            mvsdk.CameraSetAeState(self.fast_camera, 0)
            mvsdk.CameraSetExposureTime(self.fast_camera, 8000)  # 8ms
            mvsdk.CameraPlay(self.fast_camera)
            self.allocate_frame_buffer(cap)
            rospy.loginfo(f'Width: {cap.sResolutionRange.iWidthMax} x Height {cap.sResolutionRange.iHeightMax}')
            
        except mvsdk.CameraException as e:
            rospy.logerr(f"Camera setup failed: {e}")
            rospy.signal_shutdown("Camera setup failed.")

    def allocate_frame_buffer(self, cap):
        buffer_size = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if cap.sIspCapacity.bMonoSensor else 3)
        self.frame_buffer = mvsdk.CameraAlignMalloc(buffer_size, 16)

    def main_loop(self):
        while not rospy.is_shutdown():
            frame = self.capture_frame()
            if frame is not None:
                self.publish_frame(frame)
            else:
                self.missed_frames += 1
                rospy.logerr("Frame capture failed.")
            self.rate.sleep()
            if rospy.is_shutdown():
                rospy.loginfo(f"Missed {self.missed_frames} frames.")
                rospy.loginfo("Shutting down...")
                self.cleanup()

    def capture_frame(self):
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(self.fast_camera, 200)
            mvsdk.CameraImageProcess(self.fast_camera, pRawData, self.frame_buffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(self.fast_camera, pRawData)

            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(self.frame_buffer)
            return np.frombuffer(frame_data, dtype=np.uint8).reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
        except mvsdk.CameraException as e:
            rospy.logerr(f"Frame capture failed: {e}")
            return None

    def publish_frame(self, frame):
        camera_info = self.camera_info_manager.getCameraInfo()
        camera_info.header.stamp = rospy.Time.now()
        camera_info.header.frame_id = self.camera_manager
        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        ros_image.header.stamp = rospy.Time.now()
        ros_image.header.frame_id = self.camera_manager

        self.camera_info_publisher.publish(camera_info)
        self.image_pub.publish(ros_image)

    def cleanup(self):
        if self.fast_camera:
            mvsdk.CameraUnInit(self.fast_camera)
            self.fast_camera = None
            mvsdk.CameraAlignFree(self.frame_buffer)
            self.frame_buffer = None
        cv2.destroyAllWindows()

if __name__ == '__main__':
    camera_node = CameraNode()
    try:
        camera_node.main_loop()
    except rospy.ROSInterruptException:
        pass
    finally:
        camera_node.cleanup()
