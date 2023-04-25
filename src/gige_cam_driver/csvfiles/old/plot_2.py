import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime
import matplotlib.patches as patches
from scipy.signal import resample



    
def get_camera_name(string_cam):
    try:
        camera_name = "_".join(string_cam.split('/')[-1].split('_')[:2])
        complete_filename = string_cam.split('/')[-1].split('.')[0]
        return [camera_name, complete_filename]
    except Exception as e:
        return None


def csv_plotting_v2(csv_file, ldv_file):
    df_ldv = pd.read_csv(ldv_file)
    time_data_ldv = df_ldv['Time']
    disp_data_ldv = df_ldv['Distance']
    # print('length of time_data_ldv: ', len(time_data_ldv))
    # print('length of disp_data_ldv: ', len(disp_data_ldv))

    # # convert time_data_ldv to Unix timestamp
    time_data_ldv = [datetime.strptime(d, "%Y-%m-%d %H:%M:%S.%f").timestamp() for d in time_data_ldv]

    fs_ldv = int(round(1 / np.mean(np.diff(time_data_ldv)),0))
    print("Original sampling frequency: ", fs_ldv)
    fs_desired = 100
    
    # calculate downsampling factor
    factor = int(round(fs_ldv/fs_desired, 0))
    
    print('Downsampling factor: ', factor)
    # Downsample disp_data_ldv and time_data_ldv
    # disp_data_ldv = resample(disp_data_ldv, len(disp_data_ldv)//factor)
    # time_data_ldv_downsampled = resample(time_data_ldv, len(time_data_ldv)//factor)

    # Calculate new sampling interval and timestamps for downsampled signal
    delta_t = 1 / fs_desired
    time_data_ldv_downsampled = time_data_ldv[::factor]
    disp_data_ldv_downsampled = disp_data_ldv[::factor]
    print('Length before downsampling: ', len(disp_data_ldv))
    print('Length after downsampling : ', len(disp_data_ldv_downsampled))

    # Calculate actual sampling frequency of downsampled signal
    time_diffs = np.diff(time_data_ldv_downsampled)
    fs_ldv_downsampled = int(round(1 / np.mean(time_diffs),0))

    # print('fs_ldv after downsampling: ', fs_ldv_downsampled)
    
    # print('Length of time_data_ldv_downsampled: ', len(time_data_ldv_downsampled))
    # print('Length of disp_data_ldv_downsampled: ', len(disp_data_ldv_downsampled))
    
    # print('Length of time_data_ldv: ', len(time_data_ldv))
    # print('Length of disp_data_ldv: ', len(disp_data_ldv))

    

    # # convert downsampled time data back to original format
    # time_data_ldv_downsampled = [datetime.fromtimestamp(d) for d in time_data_ldv_downsampled]


    # # convert 'disp_data_ldv' values from str to numeric
    # disp_data_ldv = pd.to_numeric(disp_data_ldv, errors='coerce')
    # disp_data_ldv = np.asarray(disp_data_ldv)
    


    
    
    

    camera, path = get_camera_name(csv_file)
    data = pd.read_csv(csv_file)
    jpg_file = csv_file.replace('.csv', '.png')
    # print(csv_file)
    first_underscore = csv_file.index('_')
    second_underscore = csv_file.index('_', first_underscore + 1)
    third_underscore = csv_file.index('_', second_underscore + 1)
    duration = csv_file[second_underscore+1:third_underscore]
    # print('first_underscore',first_underscore)
    # print('second_underscore',second_underscore)
    # print('third_underscore',third_underscore)
    # print(duration)
    # return
    duration = int(duration[:-1])
    # print('Duration: ', duration)
    # print('Type of Duration is: ', type(duration))


    camera_number = csv_file[first_underscore+1:second_underscore]
    # print(camera_number)  # Output: 1
        
    
    x_disp = data['field.transforms0.transform.translation.x']
    y_disp = data['field.transforms0.transform.translation.y']
    z_disp = data['field.transforms0.transform.translation.z']
    
    print("Length1:", len(x_disp), len(y_disp), len(z_disp))
    print('Length2:', len(disp_data_ldv_downsampled))
    cam_data = y_disp[:100]
    # ldv_data = disp_data_ldv_downsampled[:100]
    plt.plot(cam_data*1000 , label='y_disp', color='red')
    # plt.plot(ldv_data, color='blue')
    plt.show()
    return
    # x_disp = data['field.transforms0.transform.translation.x'] - data['field.transforms0.transform.translation.x'][0]
    # y_disp = data['field.transforms0.transform.translation.y'] - data['field.transforms0.transform.translation.y'][0]
    # z_disp = data['field.transforms0.transform.translation.z'] - data['field.transforms0.transform.translation.z'][0]
    
    x_disp = x_disp - np.mean(x_disp)
    y_disp = y_disp - np.mean(y_disp)
    z_disp = z_disp - np.mean(z_disp)
    
    
    x_disp = np.asarray(x_disp)
    y_disp = np.asarray(y_disp)
    z_disp = np.asarray(z_disp)
    
    # Plotting
    fig, axs = plt.subplots(3, 2, figsize=(10, 8), gridspec_kw={'hspace': 0.5, 'wspace': 0.5})
    # fig.subtitle('Displacement of the marker in the camera frame', fontsize=16)
    fs =100


    
    
# Time domain plot for x-axis (zoomed)
    # duration
    # print('duration',duration, 's')
    x_axs_seconds = 1
    factor = duration
    x_len = len(x_disp)
    # print('x_len',x_len)
    new_x_len = x_len//factor
    new_x_len = new_x_len * x_axs_seconds
    # print('new_x_len',new_x_len)
    ylimit_value = (np.max(y_disp) + 0.2 * np.max(y_disp))*1000
    # print('new_x_len', new_x_len)
    
    axs[0, 0].plot(x_disp[:new_x_len], label='x_disp', color='green')
    axs[0, 0].set_xlabel('Time - zoomed (s)')
    axs[0, 0].set_ylabel('x-axis displacement (mm)')
    axs[0, 0].set_ylim([-ylimit_value, ylimit_value])
    axs[0, 0].set_title('Time domain Plots')
    axs[0, 0].set_xticks(np.linspace(0, new_x_len-1, x_axs_seconds+1))
    axs[0, 0].set_xticklabels(np.arange(0, x_axs_seconds+1))
    axs[0, 0].grid(True)

    
    print('len(disp_data_ldv)',len(disp_data_ldv))
    print('len(y_disp)',len(y_disp))

    # Time domain plot for 
    # factor = 30
    x_len = len(y_disp)
    new_x_len = x_len//factor
    new_x_len = new_x_len * x_axs_seconds
    
    # fig1, axs1 = plt.subplots(3, 2, figsize=(10, 8), gridspec_kw={'hspace': 0.5, 'wspace': 0.5})

    axs[1, 0].plot(y_disp[:new_x_len]*1000, label='y_disp', color='red')
    axs[1, 0].plot(disp_data_ldv_downsampled[:new_x_len], label='x_disp', color='green')
    axs[1, 0].set_xlabel('Time - zoomed (s)')
    axs[1, 0].set_ylabel('y-axis displacement (mm)')
    axs[1, 0].set_ylim([-ylimit_value, ylimit_value])
    # axs[1, 0].set_xticks(np.linspace(0, new_x_len-1, x_axs_seconds+1))
    # axs[1, 0].set_xticklabels(np.arange(0, x_axs_seconds+1))
    axs[1, 0].grid(True)


    

    
    # Time domain plot for 
    # factor = 10
    x_len = len(z_disp)
    new_x_len = x_len//factor
    new_x_len = new_x_len * x_axs_seconds
    axs[2, 0].plot(z_disp[:new_x_len], label='z_disp', color='blue')
    axs[2, 0].set_xlabel('Time - zoomed (s)')
    axs[2, 0].set_ylabel('z-axis displacement (mm)')
    axs[2, 0].set_ylim([-ylimit_value, ylimit_value])
    axs[2, 0].set_xticks(np.linspace(0, new_x_len-1, x_axs_seconds+1))
    axs[2, 0].set_xticklabels(np.arange(0, x_axs_seconds+1))
    axs[2, 0].grid(True)


    
    # Frequency Domain Plotting
    
    fs = 100 # Sampling frequency
    N = 2048 # Number of samples
    X = np.fft.fft(x_disp, N) # DFT of signal
    Y = np.fft.fft(y_disp, N) # DFT of signal
    Z = np.fft.fft(z_disp, N) # DFT of signal
    freq = np.fft.fftfreq(N, 1/fs)
    
    # # Find index of highest magnitude
    # max_idx = np.argmax(np.abs(Y[:N//2]))

    # # Get frequency at that index
    # max_freq = freq[max_idx]
    
    max_freq = freq[np.argmax(np.abs(Y[:N//2]))]
    max_mag = np.max(np.abs(Y[:N//2]))

    # Highlight point on plot
    # axs[1, 2].plot(max_freq, np.abs(X[max_idx]), 'ro', markersize=5)
    
    
    axs[0, 1].semilogy(freq[:N//2], np.abs(X[:N//2]), color='green')
    axs[0, 1].set_xlabel('Frequency (Hz)')
    axs[0, 1].set_ylabel('Magnitude')
    axs[0, 1].set_title('Frequency Spectrum')
    axs[0, 1].set_ylim([10**(-4), 10**1])
    axs[0, 1].set_xlim([0, 10])
    axs[0, 1].grid(True)

    axs[1, 1].semilogy(freq[:N//2], np.abs(Y[:N//2]), color='red')
    axs[1, 1].set_xlabel('Frequency (Hz)')
    axs[1, 1].set_ylabel('Magnitude')
    axs[1, 1].set_xlim([0, 10])
    axs[1, 1].set_ylim([10**(-4), 10**1])
    axs[1, 1].grid(True)
    rect = patches.Rectangle((max_freq-0.5, 10**(-4)), 1, 10**1, linewidth=1, edgecolor='yellow', facecolor='yellow', alpha=0.5)
    axs[1, 1].add_patch(rect)

    axs[1, 1].axvline(x=max_freq, color='black', linestyle='--', linewidth=1)
    axs[1, 1].text(max_freq+1, 10**(-3), f'{max_freq:.0f} Hz', ha='center', color='black', weight='bold', verticalalignment='bottom')
    
    axs[2, 1].semilogy(freq[:N//2], np.abs(Z[:N//2]), color='blue')
    axs[2, 1].set_xlabel('Frequency (Hz)')
    axs[2, 1].set_ylabel('Magnitude')
    axs[2, 1].set_ylim([10**(-4), 10**1])
    axs[2, 1].set_xlim([0, 10])

    axs[2, 1].grid(True)
    # max_freq = 0

    fig.suptitle(f'3D Displacement using GigE Camera {camera_number} ({max_freq:.0f} Hz signal)', fontsize=16)
    
    plt.savefig(jpg_file, dpi=300)

    plt.show()

    



if __name__ == "__main__":
    # dir_name = '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/'
    
    csv_file = 'camera_1_30s_2023-04-20_06-17-00_5Hz.csv'
    ldv_csv_file = 'camera_1_30s_optoNCDT ILD1420_2023-04-20_06-17-03.572_5Hz.csv'
    # csv_file = 'camera_2_30s_2023-04-20_06-18-40_5Hz.csv'
    # csv_file = 'camera_1_30s_2023-04-20_05-57-51_0Hz.csv'
    # csv_file = 'camera_2_30s_2023-04-20_06-00-35_0Hz.csv'
    # csv_file = 'camera_1_30s_2023-04-20_06-13-09_2Hz.csv'
    # csv_file = 'camera_2_30s_2023-04-20_06-15-09_2Hz.csv'
    
    # file_name = 'camera_2_30s_2023-04-20_06-18-40_5Hz.csv'
    


    csv_plotting_v2(csv_file, ldv_csv_file)
    # ldv_plotting(ldv_csv_file)

    # fs = 210 # Sampling frequency
    # t = np.arange(0, 1, 1/fs) # Time vector
    # f1 = 50 # Signal frequency
    # f2 = 100
    # x = np.sin(2* np.pi*f1*t) + np.sin(2*np.pi*f2*t) # Signal

    # N = 2048 # Number of points for FFT
    # X = np.fft.fft(x, N)
    # freq = np.fft.fftfreq(N, 1/fs)

    # plt.figure()
    # plt.plot(freq[:N//2], np.abs(X[:N//2]))
    # plt.xlabel('Frequency (Hz)')
    # plt.ylabel('Magnitude')
    # plt.title('Frequency Spectrum')
    # plt.show()