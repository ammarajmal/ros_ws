import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

def get_camera_name(string_cam):
    try:
        camera_name = "_".join(string_cam.split('/')[-1].split('_')[:2])
        complete_filename = string_cam.split('/')[-1].split('.')[0]
        return [camera_name, complete_filename]
    except:
        return None
    
def csv_plotting_v2(csv_file):
    camera, path = get_camera_name(csv_file)
    data = pd.read_csv(csv_file)
    
    jpg_file = csv_file.replace('.csv', '.png')
    
    
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
    fig, axs = plt.subplots(3, 2, figsize=(10, 8), gridspec_kw={'hspace': 0.5, 'wspace': 0.5})
    # fig.subtitle('Displacement of the marker in the camera frame', fontsize=16)
    fig.suptitle('3D Displacement using GigE camera - 3 Hz (50sec)', fontsize=16)
    fs =100
    # Time domain plot
    # axs[0, 0].set_title('Time domain')
    # axs[0, 0].plot(x_disp, label='x_disp', color='green')
    # axs[0, 0].set_xlabel('Time (s)')
    # axs[0, 0].set_ylabel('x-axis displacement (m)')
    # axs[0, 0].set_ylim([-0.015, 0.015])
    # xticks = np.linspace(0, len(x_disp)/fs, 6)
    # xtick_labels = np.round(np.linspace(0, len(x_disp)/fs, 6), 2)
    # axs[0, 0].set_xticks(xticks)
    # axs[0, 0].set_xticklabels(xtick_labels)
        # xtick_labels = axs[0, 0].get_xticks()/100 # get current xtick values and divide by 100
    # axs[0, 0].set_xticklabels(xtick_labels) # set modified xtick labels

    
    
# Time domain plot for x-axis (zoomed)
    factor = 10
    x_len = len(x_disp)
    new_x_len = x_len//factor
    axs[0, 0].plot(x_disp[:new_x_len], label='x_disp', color='green')
    axs[0, 0].set_xlabel('Time - zoomed (s)')
    axs[0, 0].set_ylabel('x-axis displacement (mm)')
    axs[0, 0].set_ylim([-0.015, 0.015])
    axs[0, 0].set_title('Time domain Plots')
    axs[0, 0].set_xticks(np.linspace(0, new_x_len-1, 6))
    axs[0, 0].set_xticklabels(np.arange(0, 6))
    axs[0, 0].grid(True)

    


    
    # axs[1, 0].plot(y_disp, label='y_disp', color='red')
    # axs[1, 0].set_xlabel('Time (s)')
    # axs[1, 0].set_ylabel('y-axis displacement (m)')
    # axs[1, 0].set_ylim([-0.015, 0.015])

    # Time domain plot for 
    factor = 10
    x_len = len(y_disp)
    new_x_len = x_len//factor
    # print(new_x_len)
    axs[1, 0].plot(y_disp[:new_x_len], label='y_disp', color='red')
    axs[1, 0].set_xlabel('Time - zoomed (s)')
    axs[1, 0].set_ylabel('y-axis displacement (mm)')
    axs[1, 0].set_ylim([-0.015, 0.015])
    axs[1, 0].set_xticks(np.linspace(0, new_x_len-1, 6))
    axs[1, 0].set_xticklabels(np.arange(0, 6))
    axs[1, 0].grid(True)


    

    # axs[2, 0].plot(z_disp, label='z_disp', color='blue')
    # axs[2, 0].set_xlabel('Time (s)')
    # axs[2, 0].set_ylabel('z-axis displacement (m)')
    # axs[2, 0].set_ylim([-0.015, 0.015])
    
    
    # Time domain plot for 
    factor = 10
    x_len = len(z_disp)
    new_x_len = x_len//factor
    axs[2, 0].plot(z_disp[:new_x_len], label='z_disp', color='blue')
    axs[2, 0].set_xlabel('Time - zoomed (s)')
    axs[2, 0].set_ylabel('z-axis displacement (mm)')
    axs[2, 0].set_ylim([-0.015, 0.015])
    axs[2, 0].set_xticks(np.linspace(0, new_x_len-1, 6))
    axs[2, 0].set_xticklabels(np.arange(0, 6))
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

    axs[0, 1].grid(True)

    axs[1, 1].semilogy(freq[:N//2], np.abs(Y[:N//2]), color='red')
    axs[1, 1].set_xlabel('Frequency (Hz)')
    axs[1, 1].set_ylabel('Magnitude')
    # axs[1, 1].set_ylim([10**(-4), 10**2])

    axs[1, 1].grid(True)

    # axs[1, 2].annotate('Max freq: {:.1f} Hz\nMax mag: {:.1f}'.format(max_freq, max_mag), 
    #                 xy=(max_freq, max_mag), xytext=(max_freq+5, max_mag+2),
    #                 arrowprops=dict(facecolor='cyan', shrink=0.01))
    # Highlight the maximum frequency
    # axs[1, 2].axvline(max_freq, color='red', linestyle='--')
    axs[1, 1].text(max_freq, max_mag+5, f'Frequency: {max_freq:.0f} Hz', ha='left', color='red', weight='bold', verticalalignment='bottom')

    # axs[1, 1].text(max_freq, max_mag+0.5, f'Frequency: {max_freq:.0f} Hz', ha='left')
    # axs[1, 2].text(max_freq, max_mag-0.5, f'Magnitude: {max_mag:.2f}', ha='right')

# Set the position of the max freq text
    # max_freq_text = plt.gcf().texts[-2]
    
    axs[2, 1].semilogy(freq[:N//2], np.abs(Z[:N//2]), color='blue')
    axs[2, 1].set_xlabel('Frequency (Hz)')
    axs[2, 1].set_ylabel('Magnitude')
    axs[2, 1].set_ylim([10**(-4), 10**1])

    axs[2, 1].grid(True)
    plt.savefig(jpg_file, dpi=300)

    plt.show()

    



if __name__ == "__main__":
    csv_file = '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/old/backup_data_csv/camera_1_50s_2023-04-10_03-19-05_3Hz.csv'
    # csv_file = '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/camera_1_50s_2023-04-10_03-19-05_3Hz.csv'
    # csv_file = '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/camera_1_50s_2023-04-10_03-26-52_7Hz.csv'
    # csv_file = '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/camera_1_60s_2023-04-11_22-41-10_10Hz.csv'
    # csv_file = '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/camera_1_60s_2023-04-11_22-04-43_3Hz.csv'
    csv_plotting_v2(csv_file)

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