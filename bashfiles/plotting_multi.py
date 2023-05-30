import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def py_plotting_multi(files):
    colors = ['green', 'blue']
    fig, axs = plt.subplots(3, 2, figsize=(12, 8))
    
    for idx, camera_file in enumerate(files):
        try:
            cameraData = pd.read_csv(camera_file)
            print('in plotting function - camera_file name:',camera_file)
            camera_file = os.path.basename(camera_file)

            record_cam_number = camera_file.split('_cam')[-1].split('.csv')[0]
            record_duration = camera_file.split('_')[2][:-1]

            print("Recoded Details:")
            print("Camera Number: ", record_cam_number)
            print("Record Duration: ", record_duration, 's')
            
            # loading time data from camera
            ti = cameraData['%time']


            # loading displacement data from camera
            xDisplacementsCamera = cameraData['field.transforms0.transform.translation.x']
            yDisplacementsCamera = cameraData['field.transforms0.transform.translation.y']
            zDisplacementsCamera = cameraData['field.transforms0.transform.translation.x']

            xDisplacementsCamera = (xDisplacementsCamera - np.mean(xDisplacementsCamera)) * 1000 
            yDisplacementsCamera = (yDisplacementsCamera - np.mean(yDisplacementsCamera)) * 1000 
            zDisplacementsCamera = (zDisplacementsCamera - np.mean(zDisplacementsCamera)) * 1000 

            fs = 100
            ts = 1/fs
            time_cam = np.arange(len(yDisplacementsCamera)) * ts

            # Compute the frequency axis for the DFT
            f_axis_len = len(yDisplacementsCamera)
            f_axis = np.fft.fftshift(np.fft.fftfreq(f_axis_len, 1/fs))
            # f_axis_positive = f_axis[(f_axis >= 0.1) & (f_axis <= 12)]
            f_axis_positive = f_axis[(f_axis >= 0)]
            # print('length of f_axis_positive', len(f_axis_positive))
            # return

            # Compute the DFT of the signal (camera)
            dft_x = np.fft.fft(xDisplacementsCamera)
            dft_y = np.fft.fft(yDisplacementsCamera)
            dft_z = np.fft.fft(zDisplacementsCamera)

            # Shift the DFT to center it on zero frequency
            dft_shifted_x = np.fft.fftshift(dft_x)
            dft_shifted_positive_x = abs(dft_shifted_x)[(f_axis >= 0.1) & (f_axis <= 12)]

            dft_shifted_y = np.fft.fftshift(dft_y)
            dft_shifted_positive_y = abs(dft_shifted_y)[(f_axis >= 0.1) & (f_axis <= 12)]

            dft_shifted_z = np.fft.fftshift(dft_z)
            dft_shifted_positive_z = abs(dft_shifted_z)[(f_axis >= 0.1) & (f_axis <= 12)]
            print(type(xDisplacementsCamera.values))
            # Plotting
            # Plot the camera time domain signal (x, y, z displacements)
            axs[0, 0].plot(time_cam[:2*fs], xDisplacementsCamera.values[:2*fs], label=f'x Displacements - Cam{record_cam_number}', linewidth=1, color=colors[idx])
            axs[1, 0].plot(time_cam[:2*fs], yDisplacementsCamera.values[:2*fs], label=f'y Displacements - Cam{record_cam_number}', linewidth=1, color=colors[idx])
            axs[2, 0].plot(time_cam[:2*fs], zDisplacementsCamera.values[:2*fs], label=f'z Displacements - Cam{record_cam_number}', linewidth=1, color=colors[idx])

            axs[0, 1].semilogy(f_axis_positive, dft_shifted_positive_x, color=colors[idx], label=f'x Frequency - Cam{record_cam_number}')
            axs[1, 1].semilogy(f_axis_positive, dft_shifted_positive_y, color=colors[idx], label=f'y Frequency - Cam{record_cam_number}')
            axs[2, 1].semilogy(f_axis_positive, dft_shifted_positive_z, color=colors[idx], label=f'z Frequency - Cam{record_cam_number}')
        except Exception as e:
            print(f'Error plotting file {camera_file}: {e}')

    for i in range(3):
        axs[i, 0].set_xlabel('Time (s)')
        axs[i, 0].set_ylabel('Displacement (mm)')
        axs[i, 0].set_title(f'{["x", "y", "z"][i]} - Time', fontweight='normal')
        axs[i, 0].grid(True)
        axs[i, 0].legend()

        axs[i, 1].set_xlabel('Frequency (Hz)')
        axs[i, 1].set_ylabel('Magnitude (log scale)')
        axs[i, 1].set_title(f'{["x", "y", "z"][i]} - Frequency', fontweight='normal')
        axs[i, 1].grid(True)
        axs[i, 1].legend()

    plt.tight_layout()
    plt.show()


file = ['/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/cameras_12_10s_2023-05-30_13-44-04_cam1.csv',
        '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/cameras_12_10s_2023-05-30_13-44-04_cam2.csv']
py_plotting_multi(file)
