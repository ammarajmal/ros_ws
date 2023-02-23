import numpy as np 
import cv2
import sys
from utils import ARUCO_DICT
import time

def pose_esitmation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    ''' 
    frame = Frame to be processed from the camera
    matrix_coefficients = Intrinsic camera parameters
    distortion_coefficients = Distortion Coefficients of the camera
     '''

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT[aruco_dict_type])
    parameters = cv2.aruco.DetectorParameters_create()

    # Detect the markers in the image
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, cv2.aruco_dict, parameters=parameters,
        cameraMatrix=matrix_coefficients, distCoeff=distortion_coefficients)
    
    # If markers are detected
    if len(corners) > 0:
        for i in range(0, ids.size):
            # Estimate the pose of each marker and return the values rvec and tvec
            rvecs, tvecs, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], 0.05, matrix_coefficients,
                                                                         distortion_coefficients)
            # Draw the detected markers
            cv2.aruco.drawDetectedMarkers(frame, corners)
            
            # Draw the axis for the markers
            cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvecs[i], tvecs[i], 0.1)
    return frame

if __name__ == '__main__':
    # Load the camera parameters
    # k = np.load(calibration_matrix_path)
    # d = np.load(distortion_coefficients_path)
    matrix_coefficients = np.load('camera_matrix.npy')
    distortion_coefficients = np.load('distortion_coefficients.npy')

    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("No frame")
            break
        frame = pose_esitmation(frame, 'DICT_6X6_250', matrix_coefficients, distortion_coefficients)
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

