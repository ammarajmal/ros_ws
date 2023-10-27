# ROS Workspace

A ROS workspace for repositories used in research of my PhD studies

----------------------------------

A software package with ROS wrappers for 3D displacement measuring utilizing multiple Gigabit Ethernet cameras and ARUCO marker recognition.

USB CAM DEMO Step # 01: Run the camera node

roslaunch usb_cam usb_cam-test.launch

Step # 02: Calibrate the camera

rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.020 image:=/usb_cam/image_raw camera:=/usb_cam --k-coefficients=2 --fix-principal-point i --fix-aspect-ratio

Step # 03: Detect ARUCO Marker using aruco_ros
roslaunch aruco_ros single.launch markerSize:=0.08 eye:="right"


Print ARUCO Makers:
https://tn1ck.github.io/aruco-print/
