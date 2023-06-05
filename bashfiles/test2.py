# #!/usr/env python3
# import os
# camera_file = '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/camera_2_10s_2023-05-24_23-30-09.csv'

# # print('in plotting function - camera_file name:',camera_file)
# camera_file = os.path.basename(camera_file)
# print('basefile name:',camera_file)
# print("\033[38;5;202mHello, orange!\033[0m")
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from plotting import py_plotting, py_plotting_double, py_plotting_multi

file=  ['/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/cameras_12_10s_2023-05-30_13-44-04_cam1.csv', 
             '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/cameras_12_10s_2023-05-30_13-44-04_cam2.csv']

print('Number of CSV files: ', len(file))
print('CSV Files: ', file)



camera_file = file[0]
camera_file2 = file[1]
# Read the data
camera_data1 = pd.read_csv(camera_file)
camera_data2 = pd.read_csv(camera_file2)
fs_cam = 100


# Camera 1 data
camera_time1 = camera_data1["%time"]
xdisp_camera1 = camera_data1["field.transforms0.transform.translation.x"]
ydisp_camera1 = camera_data1["field.transforms0.transform.translation.y"]
zdisp_camera1 = camera_data1["field.transforms0.transform.translation.z"]

xdisp_camera1 = (np.mean(xdisp_camera1) - xdisp_camera1) * 1000
ydisp_camera1 = (np.mean(ydisp_camera1) - ydisp_camera1) * 1000
zdisp_camera1 = (np.mean(zdisp_camera1) - zdisp_camera1) * 1000

# Camera 2 data
camera_time2 = camera_data2["%time"]
xdisp_camera2 = camera_data2["field.transforms0.transform.translation.x"]
ydisp_camera2 = camera_data2["field.transforms0.transform.translation.y"]
zdisp_camera2 = camera_data2["field.transforms0.transform.translation.z"]

xdisp_camera2 = (xdisp_camera2.mean() - xdisp_camera2) * 1000
ydisp_camera2 = (ydisp_camera2.mean() - ydisp_camera2) * 1000
zdisp_camera2 = (zdisp_camera2.mean() - zdisp_camera2) * 1000


start_time = max([camera_time1[0], camera_time2[0]])
end_time = min([camera_time1.iloc[-1], camera_time2.iloc[-1]])

camera1_indices = np.logical_and(camera_time1 >= start_time, camera_time1 <= end_time)
camera2_indices = np.logical_and(camera_time2 >= start_time, camera_time2 <= end_time)

print('start time:', start_time)
print('end time:', end_time)
print('camera 1 time:', camera_time1[0])
print('camera 2 time:', camera_time2[0])

print('len(camera_time1): ', len(camera_time1))
print('len(camera_time2): ',len(camera_time2))