#!/usr/bin/env python3
import rospy
import mvsdk
import numpy as np
from gige_cam_driver import  (CameraInit, CameraException, CameraGetCapability, CameraImageProcess,
						      CAMERA_MEDIA_TYPE_MONO8, CameraSetIspOutFormat, CameraReleaseImageBuffer,
							  CAMERA_MEDIA_TYPE_BGR8, CameraAlignMalloc, CameraGetImageBuffer, c_ubyte,
							  CameraSetTriggerMode, CameraSetAeState, CameraAlignFree, CAMERA_STATUS_TIME_OUT,
							  CameraSetExposureTime, CameraPlay, CameraUnInit, CameraEnumerateDevice)
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge
import cv2


bridge = CvBridge()
class Camera(object):
	def __init__(self):
		self.pFrameBuffer = 0
		self.quit = False

		self.DevInfo = None
		self.hCamera = 0
		self.cap = None
		rospy.init_node('camera_node', anonymous=False)
		self.image_publisher = rospy.Publisher('~image_raw', Image, queue_size=10)
		self.camera_info_publisher = rospy.Publisher('~camera_info', CameraInfo, queue_size=10)
		self.dev_id           = rospy.get_param("~device_id", 0)
		self.calibration_file = rospy.get_param("~calibration_file", 0)
		self.camera_manager   = rospy.get_param("~camera_manager", 0)
		self.camera_info_manager = CameraInfoManager(cname=self.camera_manager,
                                   				     url='file://' + self.calibration_file,
                                         			 namespace=self.camera_manager)
		self.camera_info_manager.loadCameraInfo()

	def open(self):
		if self.hCamera > 0:
			return True
		hCamera = 0
		try:
			hCamera = mvsdk.CameraInit(self.DevInfo, -1, -1)
		except mvsdk.CameraException as e:
			print(f"CameraInit Failed({e.error_code}): {e.message}")
			return False
		cap = mvsdk.CameraGetCapability(hCamera)
		monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
		if monoCamera:
			mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
		else:
			mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)
		# Switch the camera mode to continuous acquisition
		mvsdk.CameraSetTriggerMode(hCamera, 0)

        # Manually set exposure time to 30ms
		mvsdk.CameraSetAeState(hCamera, 0)
		mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)
        # Start the internal image acquisition thread of the SDK
		mvsdk.CameraPlay(hCamera)
  
		
        # Calculate the size of the RGB buffer needed, here directly allocate according to the maximum resolution of the camera.
		FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)

        # Allocate RGB buffer to store the image output by ISP
        # Note: The RAW data is transmitted from the camera to the PC. On the PC side, the software ISP is used to convert it to RGB data 
        # (if it is a monochrome camera, there is no need to convert the format, but ISP has other processing, so this buffer also needs to be allocated)
		self.pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

        # Set the acquisition callback function
        # self.quit = False
		# mvsdk.CameraSetCallbackFunction(hCamera, self.GrabCallback, 0)

		
		# FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
		# pFrameBuffer = CameraAlignMalloc(FrameBufferSize, 16)
		# CameraSetTriggerMode(hCamera, 0)
		# CameraSetAeState(hCamera, 0)
		# CameraSetExposureTime(hCamera, 10 * 1000)
		# CameraPlay(hCamera)
		self.hCamera = hCamera
		# self.pFrameBuffer = pFrameBuffer
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

            # The image data obtained in Windows is upside down, stored in BMP format. To convert to OpenCV, you need to flip it up and down
            # On Linux, the output is correct and does not need to be flipped up and down
			# import platform
			# if platform.system() == "Windows":
			# 	mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)
			
            # At this point, the image is stored in pFrameBuffer. For color cameras, pFrameBuffer=RGB data, and for black and white cameras, pFrameBuffer=8 
            # bit grayscale data
            # Convert pFrameBuffer to OpenCV image format for subsequent algorithm processing
			frame_data = (c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
			frame = np.frombuffer(frame_data, dtype=np.uint8)
			frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 else 3) )

			return frame
		except CameraException as e:
			if e.error_code != CAMERA_STATUS_TIME_OUT:
				print(f"CameraGetImageBuffer failed({e.error_code}): {e.message}")
			return None
	@mvsdk.method(mvsdk.CAMERA_SNAP_PROC)
	def GrabCallback(self, hCamera, pRawData, pFrameHead, pContext):
		FrameHead = pFrameHead[0]
		pFrameBuffer = self.pFrameBuffer

		mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
		mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

		# For Windows, the image data is upside down and stored in BMP format.
		# To convert it to OpenCV format, it needs to be flipped vertically
		# On Linux, the image is already in the correct orientation
		# if platform.system() == "Windows":
		# 	mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)

		# The image is now stored in pFrameBuffer.
		# For a color camera, pFrameBuffer contains RGB data; for a monochrome camera, it contains 8-bit grayscale data.
		# Convert the pFrameBuffer to the OpenCV image format for subsequent algorithm processing
		frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
		frame = np.frombuffer(frame_data, dtype=np.uint8)
		frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3) )

		# Resize the image to (640, 480) for display
		frame = cv2.resize(frame, (640,480), interpolation = cv2.INTER_LINEAR)
		return frame
	def run(self):
		DevList = CameraEnumerateDevice()
		nDev = len(DevList)
		if nDev < 1:
			print("No camera was found!")
			return
		for i, DevInfo in enumerate(DevList):
			print(f"{i}: {DevInfo.GetFriendlyName()} {DevInfo.GetPortType()}")
		self.DevInfo = DevList[self.dev_id]
		# print(self.DevInfo)
		if not self.open():
			print("camera not opened")
			exit(0)
		rate = rospy.Rate(100) #  (frames per second)
		rospy.loginfo(f"Camera {self.dev_id} initialized successfully")
		while not rospy.is_shutdown():
			frame = mvsdk.CameraSetCallbackFunction(self.hCamera, self.GrabCallback, 0)
			# frame = self.grab()
			if frame is not None:
				# frame = cv2.resize(frame, (640,480), interpolation = cv2.INTER_LINEAR)				
				camera_info = self.camera_info_manager.getCameraInfo()
				camera_info.header.stamp = rospy.Time.now()
				camera_info.header.frame_id = self.camera_manager
				self.camera_info_publisher.publish(camera_info)
			# Check if the frame is a valid NumPy array
			# if not isinstance(frame, np.ndarray):
			# 	rospy.logerr(f"Error: Invalid frame type: {type(frame)}")
			# 	break
			msg = bridge.cv2_to_imgmsg(frame, "bgr8")
			msg.header.frame_id = self.camera_manager
			msg.header.stamp = rospy.Time.now()
			self.image_publisher.publish(msg)
			rate.sleep()
			if rospy.is_shutdown():
				self.close()


# sourcery skip: use-contextlib-suppress
if __name__ == '__main__':
	try:
		driver = Camera()
		driver.run()
	except rospy.ROSInterruptException:
		pass		

