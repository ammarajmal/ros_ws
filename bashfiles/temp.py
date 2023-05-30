# from plotting_multi import py_plotting_multi
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.fft import fft, fftshift

file = ['/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/cameras_12_10s_2023-05-30_13-44-04_cam1.csv',
        '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/cameras_12_10s_2023-05-30_13-44-04_cam2.csv']
camera_file = file[0]
camera_file2 = file[1]
freq = '5'
def matploting(camera_file, camera_file2, freq):
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

        # Determine start and end times
        start_time = max([camera_time1[0], camera_time2[0]])
        end_time = min([camera_time1.iloc[-1], camera_time2.iloc[-1]])

        #     print((end_time - start_time) *1e-9)
        # Find indices of data within this time period
        camera1_indices = np.logical_and(camera_time1 >= start_time, camera_time1 <= end_time)
        camera2_indices = np.logical_and(camera_time2 >= start_time, camera_time2 <= end_time)

        # Trim the datasets
        ydisp_camera1 = ydisp_camera1[camera1_indices]
        camera_time1 = camera_time1[camera1_indices]

        ydisp_camera2 = ydisp_camera2[camera2_indices]
        camera_time2 = camera_time2[camera2_indices]

        # Converting into seconds from nanoseconds UNIX timestamps
        camera_time1 = (camera_time1 - camera_time1.iloc[0]) * 1e-9
        camera_time2 = (camera_time2 - camera_time2.iloc[0]) * 1e-9
        
        
        


        # Compute the freq axis for the DFT
        f_axis_len1 = len(ydisp_camera1)
        f_axis1 = np.fft.fftshift(np.fft.fftfreq(f_axis_len1, 1/fs_cam))
        f_axis_positive1 = f_axis1[(f_axis1 >= 0)]
        
        f_axis_len2 = len(ydisp_camera2)
        f_axis2 = np.fft.fftshift(np.fft.fftfreq(f_axis_len2, 1/fs_cam))
        f_axis_positive2 = f_axis2[(f_axis2 >= 0)]

        # Compute the DFT of the signal
        dft_y1 = np.fft.fft(ydisp_camera1)
        dft_y2 = np.fft.fft(ydisp_camera2)


        # Shift the DFT to center it on zero frequency
        dft_shifted_y1 = np.fft.fftshift(dft_y1)
        dft_shifted_positive_y1 = abs(dft_shifted_y1)[(f_axis1 >= 0)]

        dft_shifted_y2 = np.fft.fftshift(dft_y2)
        dft_shifted_positive_y2 = abs(dft_shifted_y2)[(f_axis2 >= 0)]
        
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
        plot_labels = ['Camera1 vs Camera 2 - y Displacements', 'Camera 1 - y Displacements', 'Camera 2 - y Displacements']
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
        
        
        plt.tight_layout()
        plt.show()

# Call the function
matploting(camera_file, camera_file2, freq)

