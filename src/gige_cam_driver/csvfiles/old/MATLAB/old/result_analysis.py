import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def plot_results(camera_file, ldv_file):
    
    # loading the data
    camera_data = pd.read_csv(camera_file)
    ldv_data = pd.read_csv(ldv_file)
    
    # extracting camera information
    camera_name = camera_file.split('_30s')[0]
    camera_number = camera_name.split('camera_')[1]
    last_underscore = camera_file.rfind('_')
    hz_value = camera_file[last_underscore+1:camera_file.find('Hz')]
    
    # laoding the camera data
    xDisplacementsCamera = camera_data['field.transforms0.transform.translation.x']
    yDisplacementsCamera = camera_data['field.transforms0.transform.translation.y']
    zDisplacementsCamera = camera_data['field.transforms0.transform.translation.x']
    
    fs_cam = 100
    ts_cam = 1/fs_cam
    time_cam = np.arange(len(yDisplacementsCamera))*ts_cam

    yDisplacementsCamera = yDisplacementsCamera - np.mean(yDisplacementsCamera)

    ldvDisplacements = ldv_data['Distance']
    
    # adjusting values
    ldvDisplacements = ldvDisplacements - np.mean(ldvDisplacements)

    fs = 1000
    downsample_freq = 100
    downsample_factor = fs//downsample_freq

    ldvDisplacements_down = ldvDisplacements[::downsample_factor]
    fs_down = fs//downsample_factor

    ts = 1/fs_down
    time = np.arange(len(ldvDisplacements_down))*ts

    # Compute the DFT of the downsampled signal (ldv)
    N_ldv = len(ldvDisplacements_down)
    dft_ldv = np.fft.fft(ldvDisplacements_down)

    # Compute the frequency axis for the DFT
    f_ldv = np.fft.fftfreq(N_ldv, d=1/fs_down)

    # Shift the DFT to center it on zero frequency
    dft_shifted_ldv = np.fft.fftshift(dft_ldv)

    # Compute the DFT of the downsampled signal (camera)
    N_cam = len(yDisplacementsCamera)
    dft_cam = np.fft.fft(yDisplacementsCamera)

    # Compute the frequency axis for the DFT
    f_cam = np.fft.fftfreq(N_cam, d=1/fs_cam)

    # Shift the DFT to center it on zero frequency
    dft_shifted_cam = np.fft.fftshift(dft_cam)

    # plotting
    fig, axs = plt.subplots(2, 3, figsize=(14, 8))
    fig.suptitle(f'Displacement Measurement - Camera {camera_number} ({hz_value}Hz)')

    axs[0, 0].plot(time[:fs_down], ldvDisplacements_down[:fs_down])
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('Displacements (mm)')
    axs[0, 0].set_title('LDV - Time')

    axs[0, 1].plot(time_cam[:100], yDisplacementsCamera[:100]*1000/2, color='red')
    axs[0, 1].set_xlabel('Time (s)')
    axs[0, 1].set_ylabel('Displacements (mm)')
    axs[0, 1].set_title('Camera - Time')
    


if __name__ == '__main__':
        # reading csv files
    camera_file = '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/old/MATLAB/old/camera_1_30s_2023-04-20_06-13-09_2Hz.csv'
    ldv_file = '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/old/MATLAB/old/protocol_optoNCDT ILD1420_2023-04-20_06-13-08.015_CAM1_2Hz_30s_.csv'
    plot_results(camera_file, ldv_file)

