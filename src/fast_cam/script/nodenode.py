#!/usr/bin/env python
# coding=utf-8

import rospy
import cv2
import numpy as np
import mvsdk
import platform
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

def main_loop(image_pub, bridge):
    # Enumerate cameras
    DevList = mvsdk.CameraEnumerateDevice()
    nDev = len(DevList)
    if nDev < 1:
        rospy.loginfo("No camera was found!")
        return

    for i, DevInfo in enumerate(DevList):
        rospy.loginfo("{}: {} {}".format(i, DevInfo.GetFriendlyName(), DevInfo.GetPortType()))

    i = 0 if nDev == 1 else int(input("Select camera: "))
    DevInfo = DevList[i]
    rospy.loginfo(DevInfo)

    # Open the camera
    try:
        hCamera = mvsdk.CameraInit(DevInfo, -1, -1)
    except mvsdk.CameraException as e:
        rospy.logerr("CameraInit Failed({}): {}".format(e.error_code, e.message))
        return

    # Get camera capability description
    cap = mvsdk.CameraGetCapability(hCamera)

    # Determine whether it is a black and white camera or a color camera
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

    # Set output format
    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    # Switch camera mode to continuous acquisition
    mvsdk.CameraSetTriggerMode(hCamera, 0)

    # Manual exposure, exposure time 30ms
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 30 * 1000)

    # Start the SDK internal image acquisition thread
    mvsdk.CameraPlay(hCamera)

    # Calculate the size of the RGB buffer required
    FrameBufferSize = cap.sResolutionRange.iWidthMax * cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)

    # Allocate the RGB buffer
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    while not rospy.is_shutdown():
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
            mvsdk.CameraImageProcess(hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)

            frame_data = (mvsdk.c_ubyte * FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape((FrameHead.iHeight, FrameHead.iWidth, 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
            frame = cv2.resize(frame, (640, 480), interpolation=cv2.INTER_LINEAR)

            # Convert to ROS Image message and publish
            if monoCamera:
                ros_image = bridge.cv2_to_imgmsg(frame, "mono8")
            else:
                ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
            image_pub.publish(ros_image)

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                rospy.logerr("CameraGetImageBuffer failed({}): {}".format(e.error_code, e.message))

    # Cleanup
    mvsdk.CameraUnInit(hCamera)
    mvsdk.CameraAlignFree(pFrameBuffer)

def main():
    rospy.init_node('camera_node', anonymous=True)
    image_pub = rospy.Publisher("image_raw", Image, queue_size=10)
    bridge = CvBridge()

    try:
        main_loop(image_pub, bridge)
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
