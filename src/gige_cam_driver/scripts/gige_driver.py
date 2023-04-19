#!/usr/bin/env python
#coding=utf-8
import cv2
import numpy as np
import platform
import time
import rospy
from gige_cam_driver import  (CameraInit, CameraException, CameraGetCapability, CameraImageProcess,
						      CAMERA_MEDIA_TYPE_MONO8, CameraSetIspOutFormat, CameraReleaseImageBuffer,
							  CAMERA_MEDIA_TYPE_BGR8, CameraAlignMalloc, CameraGetImageBuffer, c_ubyte,
							  CameraSetTriggerMode, CameraSetAeState, CameraAlignFree, CAMERA_STATUS_TIME_OUT,
							  CameraSetExposureTime, CameraPlay, CameraUnInit, CameraEnumerateDevice)
from sensor_msgs.msg import Image, CameraInfo
from camera_info_manager import CameraInfoManager
from cv_bridge import CvBridge


def publish_image(publisher, frame, bridge):
    msg = bridge.cv2_to_imgmsg(frame, "bgr8")
    publisher.publish(msg)

def main_loop():
    rospy.init_node('camera_node', anonymous=True)
    publisher = rospy.Publisher('camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()

    # Original script code with some modifications
    DevList = mvsdk.CameraEnumerateDevice()
    nDev = len(DevList)
    if nDev < 1:
        print("No camera was found!")
        return

    for i, DevInfo in enumerate(DevList):
        print("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))
    i = 0 if nDev == 1 else int(input("Select camera: "))
    DevInfo = DevList[i]
    print(DevInfo)

    hCamera = 0
    try:
        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    except mvsdk.CameraException as e:
        print("CameraInit Failed({}): {}".format(e.error_code, e.message) )
        return

    cap = mvsdk.CameraGetCapability(hCamera)

    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    mvsdk.CameraSetTriggerMode(hCamera, 0)

    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 10 * 1000)

    mvsdk.CameraPlay(hCamera)

    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    while not rospy.is_shutdown() and (cv2.waitKey(1) & 0xFF) != ord('q'):
        start_time = time.time()
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)

            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))

            frame = cv2.resize(frame, (640,480), interpolation = cv2.INTER_LINEAR)
            end_time = time.time()
            print("FPS: ", 1/(end_time-start_time))
            cv2.imshow("Press q to end", frame)
            publish_image(publisher, frame, bridge)

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                            print("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))

    # Close the camera
    mvsdk.CameraUnInit(hCamera)

    # Free frame buffer
    mvsdk.CameraAlignFree(pFrameBuffer)

def main():
    try:
        main_loop()
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()