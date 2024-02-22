#!/usr/bin/env python3
import rospy
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
		self.DevInfo = None
		self.hCamera = 0
		self.cap = None
		self.pFrameBuffer = 0
		rospy.init_node('camera_node', anonymous=False)
		self.image_publisher = rospy.Publisher('~image_raw', Image, queue_size=10)
		self.camera_info_publisher = rospy.Publisher('~camera_info', CameraInfo, queue_size=10)
		self.dev_id           = rospy.get_param("~device_id", 0)
		self.calibration_file = rospy.get_param("~calibration_file", 0)
		self.camera_manager   = rospy.get_param("~camera_manager", 0)
		self.camera_info_manager = CameraInfoManager(cname=self.camera_manager,url=f'file://{self.calibration_file}' ,namespace=self.camera_manager)
		# self.camera_info_manager = CameraInfoManager(cname=self.camera_manager,url='file://' + self.calibration_file,namespace=self.camera_manager)
		self.camera_info_manager.loadCameraInfo()

	def open(self):
		if self.hCamera > 0:
			return True
		hCamera = 0
		try:
			hCamera = CameraInit(self.DevInfo, -1, -1)
		except CameraException as e:
			print(f"CameraInit Failed({e.error_code}): {e.message}")
			# print("CameraInit Failed({}): {}".format(e.error_code, e.message) )
			return False
		cap = CameraGetCapability(hCamera)
		monoCamera = (cap.sIspCapacity.bMonoSensor != 0)
		if monoCamera:
			CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8)
		else:
			CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8)
		FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
		pFrameBuffer = CameraAlignMalloc(FrameBufferSize, 16)
		CameraSetTriggerMode(hCamera, 0)
		CameraSetAeState(hCamera, 0)
		CameraSetExposureTime(hCamera, 8 * 1000)
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
			frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 else 3) )
			return frame
		except CameraException as e:
			if e.error_code != CAMERA_STATUS_TIME_OUT:
				print(f"CameraGetImageBuffer failed({e.error_code}): {e.message}")
				# print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message) )
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
		rate = rospy.Rate(100) #  (frames per second)
		rospy.loginfo(f"Camera {self.dev_id} initialized successfully")
		while not rospy.is_shutdown():
			frame = self.grab()
			if frame is not None:
				frame = cv2.resize(frame, (640,480), interpolation = cv2.INTER_LINEAR)
				camera_info = self.camera_info_manager.getCameraInfo()
				camera_info.header.stamp = rospy.Time.now()
				camera_info.header.frame_id = self.camera_manager
				self.camera_info_publisher.publish(camera_info)
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


import contextlib
if __name__ == '__main__':
	with contextlib.suppress(rospy.ROSInterruptException):
		driver = Camera()
		driver.run()		

