#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy import signal


def get_camera_name(string_cam):
    try:
        camera_name = "_".join(string_cam.split('/')[-1].split('_')[:2])
        complete_filename = string_cam.split('/')[-1].split('.')[0]
        return [camera_name, complete_filename]
    except:
        return None


def csv_plotting(csv_file, upsample_factor):
    """This function plots the data from the csv file passed as input argument"""
    fs = 200
    camera, path = get_camera_name(csv_file)
    data = pd.read_csv(csv_file)
    jpg_file = csv_file.replace('.csv', '.png')
    # Extract the columns from the csv file
    camera_name = data['field.header.frame_id']
    marker_id = data['field.transforms0.fiducial_id']
    image_seq = data['field.image_seq']
    x_disp = data['field.transforms0.transform.translation.x'] - \
        data['field.transforms0.transform.translation.x'][0]
    y_disp = data['field.transforms0.transform.translation.y'] - \
        data['field.transforms0.transform.translation.y'][0]
    z_disp = data['field.transforms0.transform.translation.z'] - \
        data['field.transforms0.transform.translation.z'][0]
    # finding the length of x_disp
    # len_disp = len(x_disp)
    time_vector = range(len(x_disp))

    # upsample the time vector
    up_factor = upsample_factor
    # upsampled_time = np.linspace(0, len(t) - 1, len(t) * up_factor)
    upsampled_time = np.linspace(
        time_vector[0], time_vector[-1], len(time_vector) * up_factor)

    # Now, upsample the displacement signals using linear interpolation.
    x_disp_up = np.interp(upsampled_time, time_vector, x_disp)
    y_disp_up = np.interp(upsampled_time, time_vector, y_disp)
    z_disp_up = np.interp(upsampled_time, time_vector, z_disp)

    f_x_disp, Pxx = signal.csd(x_disp_up, x_disp_up, fs, nfft=2048)
    f_y_disp, Pyy = signal.csd(y_disp_up, y_disp_up, fs, nfft=2048)
    f_z_disp, Pzz = signal.csd(z_disp_up, z_disp_up, fs, nfft=2048)
    
    # Define the cutoff frequency for the filter
    cutoff_freq = fs / 2

    # Apply a low-pass filter to the frequency response of x_disp
    b, a = signal.butter(4, cutoff_freq, 'lowpass', fs=fs*up_factor)
    filtered_Pxx = signal.filtfilt(b, a, Pxx)

    fig, axis_plot = plt.subplots(nrows=3, ncols=2)

    # set the title of the figure
    fig.suptitle(f'3D displacement using ARUCO marker detection - {camera}')

    # Plotting the x, y, z displacement
    axis_plot[0, 0].plot([t/100 for t in time_vector], x_disp, 'r')
    axis_plot[0, 0].set_title('X displacement', loc='left', fontdict={
                              'fontsize': 14, 'fontweight': 'bold'})
    axis_plot[0, 0].set_xlabel('Time (sec)')

    axis_plot[1, 0].plot([t/100 for t in time_vector], y_disp, 'g')
    axis_plot[1, 0].set_title('Y displacement', loc='left', fontdict={
                              'fontsize': 14, 'fontweight': 'bold'})
    axis_plot[1, 0].set_xlabel('Time (sec)')

    axis_plot[2, 0].plot([t/100 for t in time_vector], z_disp, 'b')
    axis_plot[2, 0].set_title('Z displacement', loc='left', fontdict={
                              'fontsize': 14, 'fontweight': 'bold'})
    axis_plot[2, 0].set_xlabel('Time (sec)')

    axis_plot[0, 0].grid(True)
    axis_plot[1, 0].grid(True)
    axis_plot[2, 0].grid(True)
    
    # Plotting the frequency response
    axis_plot[0, 1].semilogy(f_x_disp, np.abs(Pxx))
    axis_plot[0, 1].set_title('X displacement frequency response', loc='left', fontdict={
                            'fontsize': 14, 'fontweight': 'bold'})
    axis_plot[0, 1].set_xlabel('Frequency (Hz)')

    axis_plot[1, 1].semilogy(f_y_disp, np.abs(Pyy))
    axis_plot[1, 1].set_title('Y displacement frequency response', loc='left', fontdict={
                            'fontsize': 14, 'fontweight': 'bold'})
    axis_plot[1, 1].set_xlabel('Frequency (Hz)')

    axis_plot[2, 1].semilogy(f_z_disp, np.abs(Pzz))
    axis_plot[2, 1].set_title('Z displacement frequency response', loc='left', fontdict={
                            'fontsize': 14, 'fontweight': 'bold'})
    axis_plot[2, 1].set_xlabel('Frequency (Hz)')

    axis_plot[0, 1].grid(True)
    axis_plot[1, 1].grid(True)
    axis_plot[2, 1].grid(True)

    # Saving the plot with a given name
    fig.savefig(jpg_file)
    print('Close the figure to continue..')
    plt.show()


if __name__ == '__main__':
    FILE1 = '/home/agcam/ros_ws/src/gige_cam_driver/csvfiles/backup/camera_1_15s_2023-04-05_03-56-57.csv'
    FILE2 = '/home/agcam/ros_ws/src/gige_cam_driver/csvfiles/backup/camera_1_50s_2023-04-05_08-44-37.csv'
    FILE3 = '/home/agcam/ros_ws/src/gige_cam_driver/csvfiles/backup/camera_1_55s_2023-04-05_06-56-23.csv'
    csv_plotting(FILE1, 200)
    csv_plotting(FILE2, 200)
    csv_plotting(FILE3, 200)
