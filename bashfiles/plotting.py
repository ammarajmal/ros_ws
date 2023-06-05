import pandas as pd
import numpy as np
import os
import getpass

import matplotlib.pyplot as plt
username = getpass.getuser()

def py_plotting(camera_file):
    try:
        cameraData = pd.read_csv(camera_file)
        # print('In plotting function - camera_file name:', camera_file)
        camera_file = os.path.basename(camera_file)

        record_cam_number = camera_file.split('_')[1]
        record_duration = camera_file.split('_')[2][:-1]

        print("Recorded Details:")
        print("Camera Number: ", record_cam_number)
        print("Record Duration: ", record_duration, 's')

        # Loading displacement data from camera
        xDisplacementsCamera = cameraData['field.transforms0.transform.translation.x']
        yDisplacementsCamera = cameraData['field.transforms0.transform.translation.y']
        zDisplacementsCamera = cameraData['field.transforms0.transform.translation.x']

        # Preprocess the displacement data
        def preprocess_displacements(displacements):
            return (displacements - np.mean(displacements)) * 1000

        xDisplacementsCamera = preprocess_displacements(xDisplacementsCamera)
        yDisplacementsCamera = preprocess_displacements(yDisplacementsCamera)
        zDisplacementsCamera = preprocess_displacements(zDisplacementsCamera)

        # Scale amplitude axis of time plots based on the maximum displacement of each axis
        max_disp = np.max([np.abs(xDisplacementsCamera), np.abs(yDisplacementsCamera), np.abs(zDisplacementsCamera)])
        y_lim = [-1.2 * max_disp, 1.2 * max_disp]

        fs = 100
        ts = 1 / fs
        time_cam = np.arange(len(yDisplacementsCamera)) * ts

        # Compute the frequency axis for the DFT
        f_axis_len = len(yDisplacementsCamera)
        f_axis = np.fft.fftshift(np.fft.fftfreq(f_axis_len, 1 / fs))
        f_axis_positive = f_axis[(f_axis >= 0.1)]

        # Compute the DFT of the signal (camera)
        dft_x = np.fft.fft(xDisplacementsCamera)
        dft_y = np.fft.fft(yDisplacementsCamera)
        dft_z = np.fft.fft(zDisplacementsCamera)

        # Shift the DFT to center it on zero frequency
        dft_shifted_x = np.fft.fftshift(dft_x)
        dft_shifted_positive_x = np.abs(dft_shifted_x)[(f_axis >= 0.1)]

        dft_shifted_y = np.fft.fftshift(dft_y)
        dft_shifted_positive_y = np.abs(dft_shifted_y)[(f_axis >= 0.1)]

        dft_shifted_z = np.fft.fftshift(dft_z)
        dft_shifted_positive_z = np.abs(dft_shifted_z)[(f_axis >= 0.1)]

        # Plotting
        fig, axs = plt.subplots(3, 2, figsize=(12, 8))
        fig.suptitle(f"Displacement Measurement - Camera{record_cam_number}")

        # Plot time domain signals
        axes_titles = ['x - Time', 'y - Time', 'z - Time']
        displacements = [xDisplacementsCamera, yDisplacementsCamera, zDisplacementsCamera]
        colors = ['green', 'red', 'orange']
        for i, (ax, displacement, title, color) in enumerate(zip(axs[:, 0], displacements, axes_titles, colors)):
            ax.plot(time_cam[:1 * fs], displacement.values[:1 * fs], label=f'{title[0]} Displacements',
                    linewidth=1, color=color)
            ax.set_xlabel('Time (s)')
            ax.set_ylabel('Displacement (mm)')
            ax.set_ylim(y_lim)
            ax.set_title(title, fontweight='normal')
            ax.grid(True)

        # Plot frequency domain signals
        axes_titles = ['x - Frequency', 'y - Frequency', 'z - Frequency']
        dft_shifted_positive = [dft_shifted_positive_x, dft_shifted_positive_y, dft_shifted_positive_z]
        for i, (ax, dft_shifted_pos, title, color) in enumerate(zip(axs[:, 1], dft_shifted_positive, axes_titles, colors)):
            ax.semilogy(f_axis_positive, dft_shifted_pos, color=color)
            ax.set_xlabel('Frequency (Hz)')
            ax.set_ylabel('Magnitude (log scale)')
            ax.set_title(title, fontweight='normal')
            ax.grid(True)

        image = camera_file.replace('.csv', '_cams_.png')
        print('Results saved in:', image)

        plt.subplots_adjust(hspace=0.5, wspace=0.5)
        plt.savefig(image)
        plt.show()

    except Exception as e:
        print('Error in file:', camera_file)
        print(e)
        
        
def py_plotting_double(file, exp_name):
    # try:
    camera_file = file[0]
    camera_file2 = file[1]
    list1 = os.path.basename(camera_file).split('_')[:-1]
    list1.insert(0, exp_name)
    nameFile = "_".join(list1)
    nameFile+='.png'
    xlsxFile = nameFile.replace('.png', '.xlsx')
    # Read the data
    camera_data1 = pd.read_csv(camera_file)
    camera_data2 = pd.read_csv(camera_file2)
    fs_cam = 100


    # Camera 1 data
    camera_time1 = camera_data1["%time"]
    xdisp_camera1 = camera_data1["field.transforms0.transform.translation.x"]
    ydisp_camera1 = camera_data1["field.transforms0.transform.translation.y"]
    zdisp_camera1 = camera_data1["field.transforms0.transform.translation.z"]
    # Camera 2 data
    camera_time2 = camera_data2["%time"]
    xdisp_camera2 = camera_data2["field.transforms0.transform.translation.x"]
    ydisp_camera2 = camera_data2["field.transforms0.transform.translation.y"]
    zdisp_camera2 = camera_data2["field.transforms0.transform.translation.z"]

    # Preprocess the displacement data
    def preprocess_displacements(displacements):
        return (displacements - np.mean(displacements)) * 1000

    xdisp_camera1 = preprocess_displacements(xdisp_camera1)
    ydisp_camera1 = preprocess_displacements(ydisp_camera1)
    zdisp_camera1 = preprocess_displacements(zdisp_camera1)

    xdisp_camera2 = preprocess_displacements(xdisp_camera2)
    ydisp_camera2 = preprocess_displacements(ydisp_camera2)
    zdisp_camera2 = preprocess_displacements(zdisp_camera2)


    # print('Length of  ydisp_camera1:', len(ydisp_camera1))
    # print('Length of  camera_time1:', len(camera_time1))
    # print()
    # print('Length of  ydisp_camera2:', len(ydisp_camera2))
    # print('Length of  camera_time2:', len(camera_time2))

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
    # print('Length of trimmed ydisp_camera1:', len(ydisp_camera1))
    # print('Length of trimmed camera_time1:', len(camera_time1))


    ydisp_camera2 = ydisp_camera2[camera2_indices]
    camera_time2 = camera_time2[camera2_indices]
    # print()
    # print('Length of trimmed ydisp_camera2:', len(ydisp_camera2))
    # print('Length of trimmed camera_time2:', len(camera_time2))

    # Truncate arrays to minimum length
    min_len = min(len(camera_time1), len(camera_time2))
    camera_time1 = camera_time1[:min_len]
    camera_time2 = camera_time2[:min_len]
    ydisp_camera1 = ydisp_camera1[:min_len]
    ydisp_camera2 = ydisp_camera2[:min_len]

    # print('re-adjustment')

    # print('Length of trimmed ydisp_camera1:', len(ydisp_camera1))
    # print('Length of trimmed camera_time1:', len(camera_time1))
    # print()
    # print('Length of trimmed ydisp_camera2:', len(ydisp_camera2))
    # print('Length of trimmed camera_time2:', len(camera_time2))

    # Converting into seconds from nanoseconds UNIX timestamps
    camera_time1 = (camera_time1 - camera_time1.iloc[0]) * 1e-9
    camera_time2 = (camera_time2 - camera_time2.iloc[0]) * 1e-9



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
            # Reset the indices
    camera_time1 = camera_time1.reset_index(drop=True)
    camera_time2 = camera_time2.reset_index(drop=True)
    ydisp_camera1 = ydisp_camera1.reset_index(drop=True)
    ydisp_camera2 = ydisp_camera2.reset_index(drop=True)
    # Create a dictionary with the variables
    data = {
        'camera_time1': camera_time1,
        'ydisp_camera1': ydisp_camera1,
        'camera_time2': camera_time2,
        'ydisp_camera2': ydisp_camera2
    }

    # Create a DataFrame from the dictionary
    df = pd.DataFrame(data)


    image_output_file = f'/home/{username}/Desktop/Experiment Results/{nameFile}'
    plt.subplots_adjust(hspace=0.5, wspace=0.5)
    plt.savefig(image_output_file)

    # Specify the path to save the Excel file
    output_file = f'/home/{username}/Desktop/Experiment Results/{xlsxFile}'

    # Save the DataFrame to an Excel file using the xlsx engine
    df.to_excel(output_file, index=False, engine='xlsxwriter')


    plt.tight_layout()
    plt.show()

def py_plotting_multi(file, exp_name):
    # try:
        camera_file = file[0]
        camera_file2 = file[1]
        camera_file3 = file[2]
        list1 = os.path.basename(camera_file).split('_')[:-1]
        list1.insert(0, exp_name)
        nameFile = "_".join(list1)
        nameFile+='.png'
        xlsxFile = nameFile.replace('.png', '.xlsx')
        # print(nameFile, xlsxFile)
        # return        
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
        # Camera 2 data
        camera_time2 = camera_data2["%time"]
        xdisp_camera2 = camera_data2["field.transforms0.transform.translation.x"]
        ydisp_camera2 = camera_data2["field.transforms0.transform.translation.y"]
        zdisp_camera2 = camera_data2["field.transforms0.transform.translation.z"]
        # Camera 3 data
        camera_time3 = camera_data3["%time"]
        xdisp_camera3 = camera_data3["field.transforms0.transform.translation.x"]
        ydisp_camera3 = camera_data3["field.transforms0.transform.translation.y"]
        zdisp_camera3 = camera_data3["field.transforms0.transform.translation.z"]
        
        # Preprocess the displacement data
        def preprocess_displacements(displacements):
            return (displacements - np.mean(displacements)) * 1000
        
        xdisp_camera1 = preprocess_displacements(xdisp_camera1)
        ydisp_camera1 = preprocess_displacements(ydisp_camera1)
        zdisp_camera1 = preprocess_displacements(zdisp_camera1)

        xdisp_camera2 = preprocess_displacements(xdisp_camera2)
        ydisp_camera2 = preprocess_displacements(ydisp_camera2)
        zdisp_camera2 = preprocess_displacements(zdisp_camera2)

        xdisp_camera3 = preprocess_displacements(xdisp_camera3)
        ydisp_camera3 = preprocess_displacements(ydisp_camera3)
        zdisp_camera3 = preprocess_displacements(zdisp_camera3)

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

        # Truncate arrays to minimum length
        min_len = min(len(camera_time1), len(camera_time2), len(camera_time3))
        camera_time1 = camera_time1[:min_len]
        camera_time2 = camera_time2[:min_len]
        camera_time3 = camera_time3[:min_len]
        ydisp_camera1 = ydisp_camera1[:min_len]
        ydisp_camera2 = ydisp_camera2[:min_len]
        ydisp_camera3 = ydisp_camera3[:min_len]
        
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
        # Reset the indices
        camera_time1 = camera_time1.reset_index(drop=True)
        camera_time2 = camera_time2.reset_index(drop=True)
        camera_time3 = camera_time3.reset_index(drop=True)
        ydisp_camera1 = ydisp_camera1.reset_index(drop=True)
        ydisp_camera2 = ydisp_camera2.reset_index(drop=True)
        ydisp_camera3 = ydisp_camera3.reset_index(drop=True)
        # Create a dictionary with the variables
        data = {
            'camera_time1': camera_time1,
            'ydisp_camera1': ydisp_camera1,
            'camera_time2': camera_time2,
            'ydisp_camera2': ydisp_camera2,
            'camera_time3': camera_time3,
            'ydisp_camera3': ydisp_camera3
        }

        # Create a DataFrame from the dictionary
        df = pd.DataFrame(data)
        image_output_file = f'/home/{username}/Desktop/Experiment Results/{nameFile}'
        plt.subplots_adjust(hspace=0.5, wspace=0.5)
        plt.savefig(image_output_file)

        # Specify the path to save the Excel file
        output_file = f'/home/{username}/Desktop/Experiment Results/{xlsxFile}'

        # Save the DataFrame to an Excel file using the xlsx engine
        df.to_excel(output_file, index=False, engine='xlsxwriter')
        
        plt.tight_layout()
        plt.show()


# if __name__ == '__main__':
#     file=  ['/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/cameras_12_10s_2023-05-30_13-44-04_cam1.csv', 
#              '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/cameras_12_10s_2023-05-30_13-44-04_cam2.csv']
    
#     file3 = ['/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/cameras_123_10s_2023-06-02_13-12-27_cam1.csv',
#              '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/cameras_123_10s_2023-06-02_13-12-27_cam2.csv',
#              '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/cameras_123_10s_2023-06-02_13-12-27_cam3.csv']
#     # py_plotting_double(file, exp_name='agcam')
#     py_plotting_multi(file3, exp_name='agcam')