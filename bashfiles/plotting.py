import pandas as pd
import numpy as np
import matplotlib.pyplot as plt


 

def py_plotting(camera_file):
    cameraData = pd.read_csv(camera_file)
    print(camera_file)

    
    record_cam_number  = camera_file.split('_')[1]
    record_duration = camera_file.split('_')[2][:-1]
    # record_date = camera_file.split('_')[3]
    # record_time = camera_file.split('_')[4]
    # record_hz = camera_file.split('_')[5].split('Hz')[0]
    
    print("Recoded Details:")
    print("Camera Number: ", record_cam_number)
    print("Record Duration: ", record_duration, 's')
    # print("Record Date: ", record_date)
    # print("Record Time: ", record_time)
    # print("Record Frequency: ", record_hz, 'Hz')
    
    

    # loading displacement data from camera
    xDisplacementsCamera = cameraData['field.transforms0.transform.translation.x']
    yDisplacementsCamera = cameraData['field.transforms0.transform.translation.y']
    zDisplacementsCamera = cameraData['field.transforms0.transform.translation.x']
    
    
    xDisplacementsCamera = (xDisplacementsCamera - np.mean(xDisplacementsCamera)) * 1000 /2
    yDisplacementsCamera = (yDisplacementsCamera - np.mean(yDisplacementsCamera)) * 1000 /2
    zDisplacementsCamera = (zDisplacementsCamera - np.mean(zDisplacementsCamera)) * 1000 /2

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
    fig.suptitle(f"Displacement Measurement - Camera{record_cam_number} ({record_hz}Hz)")



    # Plot the camera time domain signal (x displacement)
    axs[0, 0].plot(time_cam[0:fs], xDisplacementsCamera.values[0:fs], label='x Displacements', linewidth=1, color='green')
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('Displacement (mm)')
    axs[0, 0].set_ylim(y_lim)
    axs[0, 0].set_title('x - Time', fontweight='normal')
    axs[0, 0].grid(True)

    
    # Plot the camera time domain signal (y displacement)
    axs[1, 0].plot(time_cam[0:fs], yDisplacementsCamera.values[0:fs], label='y Displacements', linewidth=1, color='red')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('Displacement (mm)')
    axs[1, 0].set_ylim(y_lim)
    axs[1, 0].set_title('y - Time', fontweight='normal')
    axs[1, 0].grid(True)
    
    # Plot the camera time domain signal (z displacement)
    axs[2, 0].plot(time_cam[0:fs], zDisplacementsCamera.values[0:fs], label='z Displacements', linewidth=1, color='orange')
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
    plt.savefig(image, dpi=300)
    
    plt.show()

    
if __name__ == '__main__':
    
    # 2 Hz 
    camera_file = 'camera_2_30s_2023-04-24_11-10-43_2Hz.csv'
    py_plotting(camera_file)

    # 4 Hz
    camera_file = 'camera_2_30s_2023-04-24_11-13-04_4Hz.csv'
    py_plotting(camera_file)
    
    # 6 Hz
    camera_file = 'camera_2_30s_2023-04-24_11-17-55_6Hz.csv'
    py_plotting(camera_file)
    
    # 8 Hz
    camera_file = 'camera_2_30s_2023-04-24_11-19-39_8Hz.csv'
    py_plotting(camera_file)
    
    # 10 Hz - camera 2
    camera_file = 'camera_2_30s_2023-04-24_11-25-13_10Hz.csv'
    py_plotting(camera_file)
