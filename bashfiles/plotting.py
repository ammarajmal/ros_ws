import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt


 

def py_plotting(camera_file):
    try:
        cameraData = pd.read_csv(camera_file)
        print('in plotting function - camera_file name:',camera_file)
        camera_file = os.path.basename(camera_file)

        record_cam_number = camera_file.split('_cam')[-1].split('.csv')[0]
        record_duration = camera_file.split('_')[2][:-1]

        # record_date = camera_file.split('_')[3]
        # record_time = camera_file.split('_')[4]
        # record_hz = camera_file.split('_')[5].split('Hz')[0]

        print("Recoded Details:")
        print("Camera Number: ", record_cam_number)
        print("Record Duration: ", record_duration, 's')

        # loading displacement data from camera
        xDisplacementsCamera = cameraData['field.transforms0.transform.translation.x']
        yDisplacementsCamera = cameraData['field.transforms0.transform.translation.y']
        zDisplacementsCamera = cameraData['field.transforms0.transform.translation.x']


        xDisplacementsCamera = (xDisplacementsCamera - np.mean(xDisplacementsCamera)) * 1000 
        yDisplacementsCamera = (yDisplacementsCamera - np.mean(yDisplacementsCamera)) * 1000 
        zDisplacementsCamera = (zDisplacementsCamera - np.mean(zDisplacementsCamera)) * 1000 

        # Scale amplitude axis of time plots based on the maximum displacement of each axis
        max_disp = np.max([np.abs(xDisplacementsCamera), np.abs(yDisplacementsCamera), np.abs(zDisplacementsCamera)])
        y_lim = [-1.2 * max_disp, 1.2 * max_disp]

        fs = 100
        ts = 1/fs
        time_cam = np.arange(len(yDisplacementsCamera)) * ts


        # Compute the frequency axis for the DFT
        f_axis_len = len(yDisplacementsCamera)
        f_axis = np.fft.fftshift(np.fft.fftfreq(f_axis_len, 1/fs))
        f_axis_positive = f_axis[(f_axis >= 0.1) & (f_axis <= 12)]

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





        # Plotting
        fig, axs = plt.subplots(3, 2, figsize=(12, 8))
        fig.suptitle(f"Displacement Measurement - Camera{record_cam_number}")



        # Plot the camera time domain signal (x displacement)
        axs[0, 0].plot(
            time_cam[:2*fs],
            xDisplacementsCamera.values[:2*fs],
            label='x Displacements',
            linewidth=1,
            color='green',
        )
        axs[0, 0].set_xlabel('Time (s)')
        axs[0, 0].set_ylabel('Displacement (mm)')
        axs[0, 0].set_ylim(y_lim)
        axs[0, 0].set_title('x - Time', fontweight='normal')
        axs[0, 0].grid(True)


        # Plot the camera time domain signal (y displacement)
        axs[1, 0].plot(
            time_cam[:2*fs],
            yDisplacementsCamera.values[:2*fs],
            label='y Displacements',
            linewidth=1,
            color='red',
        )
        axs[1, 0].set_xlabel('Time (s)')
        axs[1, 0].set_ylabel('Displacement (mm)')
        axs[1, 0].set_ylim(y_lim)
        axs[1, 0].set_title('y - Time', fontweight='normal')
        axs[1, 0].grid(True)

        # Plot the camera time domain signal (z displacement)
        axs[2, 0].plot(
            time_cam[:2*fs],
            zDisplacementsCamera.values[:2*fs],
            label='z Displacements',
            linewidth=1,
            color='orange',
        )
        axs[2, 0].set_xlabel('Time (s)')
        axs[2, 0].set_ylim(y_lim)
        axs[2, 0].set_ylabel('Displacement (mm)')
        axs[2, 0].set_title('z - Time', fontweight='normal')
        axs[2, 0].grid(True)


        axs[0, 1].semilogy(f_axis_positive, dft_shifted_positive_x, color='green')
        axs[0, 1].set_xlabel('Frequency (Hz)')
        axs[0, 1].set_ylabel('Magnitude (log scale)')
        axs[0, 1].set_title('x - Frequency', fontweight='normal')
        axs[0, 1].grid(True)


        axs[1, 1].semilogy(f_axis_positive, dft_shifted_positive_y, color='red')
        axs[1, 1].set_xlabel('Frequency (Hz)')
        axs[1, 1].set_ylabel('Magnitude (log scale)')
        axs[1, 1].set_title('y - Frequency', fontweight='normal')
        axs[1, 1].grid(True)

        axs[2, 1].semilogy(f_axis_positive, dft_shifted_positive_z, color='orange')
        axs[2, 1].set_xlabel('Frequency (Hz)')
        axs[2, 1].set_ylabel('Magnitude (log scale)')
        axs[2, 1].set_title('z - Frequency', fontweight='normal')
        axs[2, 1].grid(True)




        image = camera_file.replace('.csv', '_cams_.png')
        print('Results saved in: ', image)


        # plt.tight_layout()
        plt.subplots_adjust(hspace=0.5, wspace=0.5)

        # Save the plot in a file called image.png
        # plt.savefig(image)

        plt.show()
    except:
        print('Error in file: ', camera_file)
        
        
def py_plotting_multi(file):
    # try:
        camera_file = file[0]
        camera_file2 = file[1]
        camera_file3 = file[2]        
        # Read the data
        camera_data1 = pd.read_csv(camera_file)
        camera_data2 = pd.read_csv(camera_file2)
        camera_data3 = pd.read_csv(camera_file3)
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
        
        # Camera 3 data
        camera_time3 = camera_data3["%time"]
        xdisp_camera3 = camera_data3["field.transforms0.transform.translation.x"]
        ydisp_camera3 = camera_data3["field.transforms0.transform.translation.y"]
        zdisp_camera3 = camera_data3["field.transforms0.transform.translation.z"]
        
        xdisp_camera3 = (xdisp_camera3.mean() - xdisp_camera3) * 1000
        ydisp_camera3 = (ydisp_camera3.mean() - ydisp_camera3) * 1000
        zdisp_camera3 = (zdisp_camera3.mean() - zdisp_camera3) * 1000

        # Determine start and end times
        start_time = max([camera_time1[0], camera_time2[0], camera_time3[0]])
        end_time = min([camera_time1.iloc[-1], camera_time2.iloc[-1], camera_time3.iloc[-1]])

        #     print((end_time - start_time) *1e-9)
        # Find indices of data within this time period
        camera1_indices = np.logical_and(camera_time1 >= start_time, camera_time1 <= end_time)
        camera2_indices = np.logical_and(camera_time2 >= start_time, camera_time2 <= end_time)
        camera3_indices = np.logical_and(camera_time3 >= start_time, camera_time3 <= end_time)

        # Trim the datasets
        ydisp_camera1 = ydisp_camera1[camera1_indices]
        camera_time1 = camera_time1[camera1_indices]

        ydisp_camera2 = ydisp_camera2[camera2_indices]
        camera_time2 = camera_time2[camera2_indices]
        
        ydisp_camera3 = ydisp_camera3[camera3_indices]
        camera_time3 = camera_time3[camera3_indices]

        # Converting into seconds from nanoseconds UNIX timestamps
        camera_time1 = (camera_time1 - camera_time1.iloc[0]) * 1e-9
        camera_time2 = (camera_time2 - camera_time2.iloc[0]) * 1e-9
        camera_time3 = (camera_time3 - camera_time3.iloc[0]) * 1e-9
        
        
        
        # Compute the freq axis for the DFT
        f_axis_len1 = len(ydisp_camera1)
        f_axis1 = np.fft.fftshift(np.fft.fftfreq(f_axis_len1, 1/fs_cam))
        f_axis_positive1 = f_axis1[(f_axis1 >= 0)]
        
        f_axis_len2 = len(ydisp_camera2)
        f_axis2 = np.fft.fftshift(np.fft.fftfreq(f_axis_len2, 1/fs_cam))
        f_axis_positive2 = f_axis2[(f_axis2 >= 0)]
        
        f_axis_len3 = len(ydisp_camera3)
        f_axis3 = np.fft.fftshift(np.fft.fftfreq(f_axis_len3, 1/fs_cam))
        f_axis_positive3 = f_axis3[(f_axis3 >= 0)]

        # Compute the DFT of the signal
        dft_y1 = np.fft.fft(ydisp_camera1)
        dft_y2 = np.fft.fft(ydisp_camera2)
        dft_y3 = np.fft.fft(ydisp_camera3)


        # Shift the DFT to center it on zero frequency
        dft_shifted_y1 = np.fft.fftshift(dft_y1)
        dft_shifted_positive_y1 = abs(dft_shifted_y1)[(f_axis1 >= 0)]

        dft_shifted_y2 = np.fft.fftshift(dft_y2)
        dft_shifted_positive_y2 = abs(dft_shifted_y2)[(f_axis2 >= 0)]
        
        dft_shifted_y3 = np.fft.fftshift(dft_y3)
        dft_shifted_positive_y3 = abs(dft_shifted_y3)[(f_axis3 >= 0)]
        
        # Creating plots
        no_seconds = 1
        plot_limit = range(no_seconds*fs_cam)
        fig, axs = plt.subplots(4, 2, figsize=(10, 8))

        fig.suptitle('Displacement Comparison')
        # print(type(xdisp_camera1))
        colors = ['red', 'blue', 'green']
        # print(len(camera_time1.values[plot_limit]), len(ydisp_camera1.values))
        axs[0, 0].plot(camera_time1.values[plot_limit], ydisp_camera1.values[plot_limit], linewidth=1, color=colors[0], label='Camera 1')
        axs[0, 0].plot(camera_time2.values[plot_limit], ydisp_camera2.values[plot_limit], linewidth=1, color=colors[1], label='Camera 2')
        axs[0, 0].plot(camera_time3.values[plot_limit], ydisp_camera3.values[plot_limit], linewidth=1, color=colors[2], label='Camera 3')
        axs[1, 0].plot(camera_time1.values[plot_limit], ydisp_camera1.values[plot_limit], linewidth=1, color=colors[0], label='Camera 1')
        axs[2, 0].plot(camera_time2.values[plot_limit], ydisp_camera2.values[plot_limit], linewidth=1, color=colors[1], label='Camera 2')
        axs[3, 0].plot(camera_time3.values[plot_limit], ydisp_camera3.values[plot_limit], linewidth=1, color=colors[2], label='Camera 3')
        
        

        axs[0, 1].semilogy(f_axis_positive1, dft_shifted_positive_y1, linewidth=1, color=colors[0], label='Camera 1')
        axs[0, 1].semilogy(f_axis_positive2, dft_shifted_positive_y2, linewidth=1, color=colors[1], label='Camera 2')
        axs[0, 1].semilogy(f_axis_positive3, dft_shifted_positive_y3, linewidth=1, color=colors[2], label='Camera 3')
        axs[1, 1].semilogy(f_axis_positive1, dft_shifted_positive_y1, linewidth=1, color=colors[0], label='Camera 1')
        axs[2, 1].semilogy(f_axis_positive2, dft_shifted_positive_y2, linewidth=1, color=colors[1], label='Camera 2')
        axs[3, 1].semilogy(f_axis_positive3, dft_shifted_positive_y3, linewidth=1, color=colors[1], label='Camera 3')
        # labels = ['Camera 1', 'Camera 2']
        plot_labels = ['Cam 1 vs Cam 2 vs Cam 3- y Displacements', 'Cam 1 - y Displacements', 'Cam 2 - y Displacements', 'Cam 3 - y Displacements']
        for i in range(4):
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
        # plt.savefig('path/to/save/figure.png')

    # except:
    #     print("Error in plotting")
    #     pass
