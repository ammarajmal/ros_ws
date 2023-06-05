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

# index of start_time in camera_time1
# Find the index of start_time in camera_time1
index_start_cam_time1 = (camera_time1 >= start_time).idxmin()

print("Index of start_time in camera_time1:", index_start_cam_time1)


#     print((end_time - start_time) *1e-9)
# Find indices of data within this time period
camera1_indices = np.logical_and(camera_time1 >= start_time, camera_time1 <= end_time)
camera2_indices = np.logical_and(camera_time2 >= start_time, camera_time2 <= end_time)

# Converting into seconds from nanoseconds UNIX timestamps
camera_time1 = (camera_time1 - camera_time1.iloc[0]) * 1e-9
camera_time2 = (camera_time2 - camera_time2.iloc[0]) * 1e-9
print('Length of Camera 1', len(camera_time1))
print('Length of Camera 2', len(camera_time2))

# Determine start and end times
print('Start camera_time1:',camera_time1[0]* 1e-9)
print('Start camera_time2:',camera_time2[0]* 1e-9)
print('start time', start_time* 1e-9)
print()
print('End camera_time1:',camera_time1.iloc[-1]* 1e-9)
print('End camera_time2:',camera_time2.iloc[-1]* 1e-9)
print('end time', end_time* 1e-9)
print()
print((end_time* 1e-9)- (start_time* 1e-9))





# # Assuming camera_time1 and camera_time2 are in nanoseconds
# camera_time1_ = pd.to_datetime(camera_time1, unit='ns')
# camera_time2_ = pd.to_datetime(camera_time2, unit='ns')

# # Format the timestamps into a specific string format
# time_format = '%Y-%m-%d %H:%M:%S'  # Customize the format as per your requirement
# camera_time1_str = camera_time1_.dt.strftime(time_format)
# camera_time2_str = camera_time2_.dt.strftime(time_format)

# # Print the human-readable timestamps
# print('Camera 1 Time:', camera_time1_str)
# print()
# print('Camera 2 Time:', camera_time2_str)
# print()





# Compute the freq axis for the DFT
f_axis_len1 = len(ydisp_camera1)
f_axis1 = np.fft.fftshift(np.fft.fftfreq(f_axis_len1, 1/fs_cam))
f_axis_positive1 = f_axis1[(f_axis1 >= 0.1)]

f_axis_len2 = len(ydisp_camera2)
f_axis2 = np.fft.fftshift(np.fft.fftfreq(f_axis_len2, 1/fs_cam))
f_axis_positive2 = f_axis2[(f_axis2 >= 0.1)]


# Compute the DFT of the signal
dft_y1 = np.fft.fft(ydisp_camera1)
dft_y2 = np.fft.fft(ydisp_camera2)


# Shift the DFT to center it on zero frequency
dft_shifted_y1 = np.fft.fftshift(dft_y1)
dft_shifted_positive_y1 = abs(dft_shifted_y1)[(f_axis1 >= 0.1)]

dft_shifted_y2 = np.fft.fftshift(dft_y2)
dft_shifted_positive_y2 = abs(dft_shifted_y2)[(f_axis2 >= 0.1)]


# Creating plots
no_seconds = 1
plot_limit = range(no_seconds*fs_cam)
fig, axs = plt.subplots(3, 2, figsize=(10, 8))

fig.suptitle('Displacement Comparison')
# print(type(xdisp_camera1))
colors = ['red', 'blue']
# print(len(camera_time1.values[plot_limit]), len(ydisp_camera1.values))
axs[0, 0].plot(camera_time1.values[plot_limit], ydisp_camera1.values[plot_limit], linewidth=1, color=colors[0], label='Camera 1')
axs[0, 0].plot(camera_time2.values[plot_limit], ydisp_camera2.values[plot_limit], linewidth=1, color=colors[1], label='Camera 2')
axs[1, 0].plot(camera_time1.values[plot_limit], ydisp_camera1.values[plot_limit], linewidth=1, color=colors[0], label='Camera 1')
axs[2, 0].plot(camera_time2.values[plot_limit], ydisp_camera2.values[plot_limit], linewidth=1, color=colors[1], label='Camera 2')



axs[0, 1].semilogy(f_axis_positive1, dft_shifted_positive_y1, linewidth=1, color=colors[0], label='Camera 1')
axs[0, 1].semilogy(f_axis_positive2, dft_shifted_positive_y2, linewidth=1, color=colors[1], label='Camera 2')
axs[1, 1].semilogy(f_axis_positive1, dft_shifted_positive_y1, linewidth=1, color=colors[0], label='Camera 1')
axs[2, 1].semilogy(f_axis_positive2, dft_shifted_positive_y2, linewidth=1, color=colors[1], label='Camera 2')
# labels = ['Camera 1', 'Camera 2']
plot_labels = ['Cam 1 vs Cam 2 - y Displacements', 'Cam 1 - y Displacements', 'Cam 2 - y Displacements']
for i in range(3):
        axs[i, 0].set_xlabel('Time (s)')
        axs[i, 0].set_ylabel('Displacement (mm)')
        axs[i, 0].set_title(plot_labels[i])
        axs[i, 0].legend(loc='center right')
        axs[i, 0].grid(True)
                
        axs[i, 1].set_xlabel('Frequency (Hz)')
        axs[i, 1].set_ylabel('Magnitude (log scale)')
        axs[i, 1].set_title(plot_labels[i])
        axs[i, 1].legend(loc='upper right')
        axs[i, 1].grid(True)

# Create a dictionary with the variables
data = {
    'camera_time1': camera_time1,
    # 'camera_time1_str': camera_time1_str,
    'ydisp_camera1': ydisp_camera1,
    'camera_time2': camera_time2,
    # 'camera_time2_str': camera_time2_str,
    'ydisp_camera2': ydisp_camera2
}

# Create a DataFrame from the dictionary
df = pd.DataFrame(data)

# Specify the path to save the Excel file
output_file = 'data.xlsx'

# Save the DataFrame to an Excel file using the xlsx engine
df.to_excel(output_file, index=False, engine='xlsxwriter')

plt.tight_layout()
plt.show()