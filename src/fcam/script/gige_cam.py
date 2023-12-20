#!/usr/bin/env python
# coding=utf-8
import platform
import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import camera_info_manager as cim
import cv2
import numpy as np
import mvsdk


def main_loop(image_pub, bridge):
    # Define framerate for Camera
    rate = rospy.Rate(125)

    # Enumerate cameras
    device_list = mvsdk.CameraEnumerateDevice()
    num_devices = len(device_list)
    if num_devices < 1:
        rospy.loginfo("No camera was found!")
        return
    device_info = device_list[0]
    rospy.loginfo(device_info)

    # Open the camera
    try:
        hCamera = mvsdk.CameraInit(device_info, -1, -1)
    except mvsdk.CameraException as e:
        rospy.logerr(
            "CameraInit Failed({}): {}".format(
                e.error_code, e.message))
        return

    # Get camera capability description
    cap = mvsdk.CameraGetCapability(hCamera)

    # Determine whether it is a black and white camera or a color camera
    monoCamera = (cap.sIspCapacity.bMonoSensor != 0)

    if monoCamera:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_MONO8)
    else:
        mvsdk.CameraSetIspOutFormat(hCamera, mvsdk.CAMERA_MEDIA_TYPE_BGR8)

    # Switch camera mode to continuous acquisition
    mvsdk.CameraSetTriggerMode(hCamera, 0)

    # Manual exposure, exposure time 30ms
    mvsdk.CameraSetAeState(hCamera, 0)
    mvsdk.CameraSetExposureTime(hCamera, 8 * 1000)

    # Start the SDK internal image acquisition thread
    mvsdk.CameraPlay(hCamera)

    # Calculate the size of the RGB buffer required
    FrameBufferSize = cap.sResolutionRange.iWidthMax * \
        cap.sResolutionRange.iHeightMax * (1 if monoCamera else 3)
    pFrameBuffer = mvsdk.CameraAlignMalloc(FrameBufferSize, 16)

    while not rospy.is_shutdown():
        try:
            pRawData, FrameHead = mvsdk.CameraGetImageBuffer(hCamera, 200)
            mvsdk.CameraImageProcess(
                hCamera, pRawData, pFrameBuffer, FrameHead)
            mvsdk.CameraReleaseImageBuffer(hCamera, pRawData)

            if platform.system() == "Windows":
                mvsdk.CameraFlipFrameBuffer(pFrameBuffer, FrameHead, 1)

            frame_data = (
                mvsdk.c_ubyte *
                FrameHead.uBytes).from_address(pFrameBuffer)
            frame = np.frombuffer(frame_data, dtype=np.uint8)
            frame = frame.reshape(
                (FrameHead.iHeight,
                 FrameHead.iWidth,
                 1 if FrameHead.uiMediaType == mvsdk.CAMERA_MEDIA_TYPE_MONO8 else 3))
            # frame = cv2.resize(
            #     frame, (640, 480), interpolation=cv2.INTER_LINEAR)

            # frame_undistort = cv2.undistort(frame, camera_matrix, dist_coeffs)
            # frame = frame_undistort
            # Convert the captured frame to a ROS Image message
            ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")

            # Publish the image
            image_pub.publish(ros_image)

            # Sleep to maintain a constant frame rate
            rate.sleep()

        except mvsdk.CameraException as e:
            if e.error_code != mvsdk.CAMERA_STATUS_TIME_OUT:
                rospy.logerr(
                    "CameraGetImageBuffer failed({}): {}".format(
                        e.error_code, e.message))

    # Cleanup
    mvsdk.CameraUnInit(hCamera)
    mvsdk.CameraAlignFree(pFrameBuffer)


def main():
    """ main function """
    rospy.init_node('camera_node', anonymous=True)
    
    # Setup CameraInfoManager
    
    
    image_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)
    bridge = CvBridge()

    try:
        main_loop(image_pub, bridge)
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
