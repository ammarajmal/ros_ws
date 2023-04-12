import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from scipy.fft import fft, fftfreq, fftshift
from scipy.fft import rfft, rfftfreq
def get_camera_name(string_cam):
    try:
        camera_name = "_".join(string_cam.split('/')[-1].split('_')[:2])
        complete_filename = string_cam.split('/')[-1].split('.')[0]
        return [camera_name, complete_filename]
    except:
        return None
    
def csv_plotting_v2(csv_file):
    fs = 200
    camera, path = get_camera_name(csv_file)
    data = pd.read_csv(csv_file)
    if 'ALIGNED' in data.columns:
        data = data.drop(columns=['ALIGNED'])

    jpg_file = csv_file.replace('.csv', '.png')
    jpg_file = jpg_file+"_"+str(fs)+"__&&&_.png"

    # camera_name = data['field.header.frame_id']
    # marker_id = data['field.transforms0.fiducial_id']
    # image_seq = data['field.image_seq']
    x_disp = data['field.transforms0.transform.translation.x']
    y_disp = data['field.transforms0.transform.translation.y']
    z_disp = data['field.transforms0.transform.translation.z']
    
    x_disp = data['field.transforms0.transform.translation.x'] - data['field.transforms0.transform.translation.x'][0]
    y_disp = data['field.transforms0.transform.translation.y'] - data['field.transforms0.transform.translation.y'][0]
    z_disp = data['field.transforms0.transform.translation.z'] - data['field.transforms0.transform.translation.z'][0]
    
    x_disp = x_disp - np.mean(x_disp)
    y_disp = y_disp - np.mean(y_disp)
    z_disp = z_disp - np.mean(z_disp)
    
    x_disp = np.asarray(x_disp)
    y_disp = np.asarray(y_disp)
    z_disp = np.asarray(z_disp)
    
    # Plotting
    fig, axs = plt.subplots(3, 3, figsize=(10, 8), gridspec_kw={'hspace': 0.5})
    # fig.subtitle('Displacement of the marker in the camera frame', fontsize=16)
    
    # Time domain plot
    axs[0, 0].set_title('Time domain')
    axs[0, 0].plot(x_disp, label='x_disp', color='green')
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('x-axis displacement (m)')
    axs[0, 0].set_ylim([-0.015, 0.015])
    
# Time domain plot for x-axis (zoomed)
    factor = 10
    x_len = len(x_disp)
    new_x_len = x_len//factor
    axs[0, 1].plot(x_disp[:new_x_len], label='x_disp', color='green')
    axs[0, 1].set_xlabel('Time - zoomed (s)')
    axs[0, 1].set_ylabel('x-axis displacement (m)')
    axs[0, 1].set_ylim([-0.015, 0.015])
    axs[0, 1].set_title('Time domain (zoomed))')

    
    axs[1, 0].plot(y_disp, label='y_disp', color='red')
    axs[1, 0].set_xlabel('Time (s)')
    axs[1, 0].set_ylabel('y-axis displacement (m)')
    axs[1, 0].set_ylim([-0.015, 0.015])

    # Time domain plot for 
    factor = 10
    x_len = len(y_disp)
    new_x_len = x_len//factor
    # print(new_x_len)
    axs[1, 1].plot(y_disp[:new_x_len], label='y_disp', color='red')
    axs[1, 1].set_xlabel('Time - zoomed (s)')
    axs[1, 1].set_ylabel('y-axis displacement (m)')
    axs[1, 1].set_ylim([-0.015, 0.015])
    

    axs[2, 0].plot(z_disp, label='z_disp', color='blue')
    axs[2, 0].set_xlabel('Time (s)')
    axs[2, 0].set_ylabel('z-axis displacement (m)')
    axs[2, 0].set_ylim([-0.015, 0.015])
    
    
    # Time domain plot for 
    factor = 10
    x_len = len(z_disp)
    new_x_len = x_len//factor
    axs[2, 1].plot(z_disp[:new_x_len], label='z_disp', color='blue')
    axs[2, 1].set_xlabel('Time - zoomed (s)')
    axs[2, 1].set_ylabel('z-axis displacement (m)')
    axs[2, 1].set_ylim([-0.015, 0.015])

    
    # Frequency domain plot
    n = len(y_disp)
    points_per_freq = n / (fs/2)
    target_idx = int(points_per_freq * 20)
    
    yf = rfft(y_disp)
    xf = rfftfreq(n, 1/fs)
    
    yf[target_idx - 1 : target_idx + 2] = 0

    # plt.plot(xf, np.abs(yf))


    # Frequency Plots
    axs[1, 2].plot(xf, np.abs(yf), label='x_disp', color='green')
    axs[1, 2].set_xlabel('x-axis Frequency (Hz)')
    axs[1, 2].set_ylabel('Magnitude')
    axs[1, 2].set_title('Frequency domain')
    # axs[0, 2].set_ylim([0, 0.01])

    # yf = fft(y_disp)
    
    
    # yf = fftshift(yf)
    # # xf = np.fft.fftshift(xf)

    # pos_freq = xf > 0
    # xf = xf[pos_freq]
    # yf = yf[pos_freq]

    # # Frequency Plots
    # axs[1, 2].plot(xf, 2.0/n * np.abs(yf), label='y_disp', color='red')
    # # axs[1, 2].plot(xf[:n//2], 2.0/n * np.abs(yf[0:n//2]), label='y_disp', color='red')
    # axs[1, 2].set_xlabel('y-axis Frequency (Hz)')
    # axs[1, 2].set_ylabel('Magnitude')
    # axs[1, 2].set_ylim([0, 0.01])

    # yf = fft(z_disp)
    # axs[2, 2].plot(xf[:n//2], 2.0/n * np.abs(yf[0:n//2]), label='z_disp', color='blue')
    # axs[2, 2].set_xlabel('z-axis Frequency (Hz)')
    # axs[2, 2].set_ylabel('Magnitude')
    # axs[2, 2].set_ylim([0, 0.01])
    

    plt.show()
    # Save the figure
    # plt.savefig(jpg_file)
        

        
        
if __name__ == '__main__':
    # Path to the csv file
    csv_file = '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/camera_1_50s_2023-04-10_03-19-05_3Hz.csv'
    # csv_file = '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/camera_1_50s_2023-04-10_03-26-52_7Hz.csv'
    # csv_file = '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/camera_1_60s_2023-04-11_22-41-10_10Hz.csv'
    csv_plotting_v2(csv_file)