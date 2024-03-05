#include "CameraApi.h" // Camera SDK's API header file

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>

using namespace cv;

unsigned char *g_pRgbBuffer; // Processed data buffer

int main()
{
    int iCameraCounts = 1;
    int iStatus = -1;
    tSdkCameraDevInfo tCameraEnumList;
    int hCamera;
    tSdkCameraCapbility tCapability; // Device description information
    tSdkFrameHead sFrameInfo;
    BYTE *pbyBuffer;
    int iDisplayFrames = 10000;
    int channel = 3;

    CameraSdkInit(1);

    // Enumerate devices and build device list
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("state = %d\n", iStatus);

    printf("count = %d\n", iCameraCounts);
    // No connected devices
    if (iCameraCounts == 0)
    {
        return -1;
    }

    // Camera initialization. After successful initialization, other camera-related operation interfaces can be called
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);

    // Initialization failed
    printf("state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        return -1;
    }

    // Get the camera's feature description structure. This structure contains the range of various parameters that can be set for the camera, which determines the parameters of related functions
    CameraGetCapability(hCamera, &tCapability);

    g_pRgbBuffer = (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);

    // Let the SDK enter the working mode, start receiving image data sent from the camera. If the current camera is in trigger mode, it needs to receive the trigger frame before updating the image
    CameraPlay(hCamera);

    /* Other camera parameter settings
       For example, CameraSetExposureTime, CameraGetExposureTime for setting/reading exposure time
       CameraSetImageResolution, CameraGetImageResolution for setting/reading resolution
       CameraSetGamma, CameraSetContrast, CameraSetGain, etc. for setting image gamma, contrast, RGB digital gain, etc.
       This example is just to demonstrate how to convert the image obtained from the SDK into OpenCV's image format, so as to call OpenCV's image processing functions for subsequent development
    */

    if (tCapability.sIspCapacity.bMonoSensor)
    {
        channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    }
    else
    {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }

    // Loop to display 10000 frames of images
    while (iDisplayFrames--)
    {
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
        {
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

            cv::Mat matImage(
                cv::Size(sFrameInfo.iWidth, sFrameInfo.iHeight),
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer);
            imshow("Opencv Demo", matImage);

            waitKey(5);

            // After successfully calling CameraGetImageBuffer, you must call CameraReleaseImageBuffer to release the obtained buffer.
            // Otherwise, the next call to CameraGetImageBuffer will hang the program, blocking until CameraReleaseImageBuffer is called in another thread to release the buffer
            CameraReleaseImageBuffer(hCamera, pbyBuffer);
        }
    }

    CameraUnInit(hCamera);
    // Note, free the buffer after deinitializing
    free(g_pRgbBuffer);

    return 0;
}
