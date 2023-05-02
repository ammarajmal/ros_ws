import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime

def parse_date_string(date_string):
    return datetime.strptime(date_string, '%Y-%m-%d %H:%M:%S.%f')
def plot_data(filename_ldv, filename_cam):
    print('entered function')
    df_ldv = pd.read_csv(filename_ldv)

    # convert 'Snapshot Time Stamp' column to datetime format
    time_data_ldv = df_ldv['Time']
    disp_data_ldv = df_ldv['Distance']

    # convert 'disp_data_ldv' values from str to numeric
    disp_data_ldv = pd.to_numeric(disp_data_ldv, errors='coerce')

    disp = disp_data_ldv - disp_data_ldv[0]
    disp = disp - np.mean(disp)
    disp = np.asarray(disp)

    time_difference = parse_date_string(time_data_ldv[1]) - parse_date_string(time_data_ldv[0])
    difference_in_seconds_ldv = time_difference.total_seconds()
    fs_ldv = 1/difference_in_seconds_ldv
    
    print(difference_in_seconds_ldv)
    print('fs_ldv: ', fs_ldv)

    fig, axs = plt.subplots(2, 3, figsize=(16, 8), gridspec_kw={'hspace': 0.5, 'wspace': 0.5})
    
    axs[0,0].set_title('Time Domain Plot (LDV)') 
    axs[0,0].plot(disp, label='Displacement', color = 'green')
    
    axs[0, 0].set_xlabel('Time (s)')
    axs[0, 0].set_ylabel('Displacement (mm)')
    axs[0, 0].grid(True)
    axs[0, 0].set_xlim(0, 2000)
    axs[0, 0].set_xticks(np.linspace(0, 2000-1, 3))
    axs[0, 0].set_xticklabels(np.arange(0, 3))
    
    N = 2048
    X = np.fft.fft(disp, N)
    freq = np.fft.fftfreq(N, 1/fs_ldv)

    max_freq_ldv = freq[np.argmax(np.abs(X[:N//2]))]
    max_mag_ldv = np.max(np.abs(X[:N//2]))

    # set x-limit to 20 Hz
    freq_mask = freq[:N//2] <= 20
    axs[1, 0].semilogy(freq[:N//2][freq_mask], np.abs(X[:N//2])[freq_mask], label='FFT (LDV)', color = 'green')
    axs[1, 0].set_xlim([0, 20])

    axs[1, 0].set_title('Frequency Spectrum (LDV)')
    axs[1, 0].set_xlabel('Frequency (Hz)')
    axs[1, 0].set_ylabel('Amplitude')
    axs[1, 0].grid(True)
    axs[1, 0].text(max_freq_ldv, max_mag_ldv+10**4, f'Frequency: {max_freq_ldv:.0f} Hz', ha='right', color='green', weight='bold', verticalalignment='center')

    
    
    data_cam = pd.read_csv(filename_cam)
    print(' len of data_cam: ', len(data_cam))
    y_disp_cam = data_cam['field.transforms0.transform.translation.y'] - data_cam['field.transforms0.transform.translation.y'][0]
    y_disp_cam = y_disp_cam - np.mean(y_disp_cam)
    # y_disp_cam = np.asarray(y_disp_cam)
    fs_cam = 1000

    upsample_factor = 10
    time_vector = range(len(y_disp_cam))
    upsampled_time = np.linspace(
                time_vector[0], time_vector[-1], len(time_vector) * upsample_factor)
    y_disp_up = np.interp(upsampled_time, time_vector, y_disp_cam)
    
    # print(' len of y_disp_up: ', len(y_disp_up))


    gain = 1333.33
    factor = 30
    y_disp_up = y_disp_up * gain
    x_len = len(y_disp_up)
    new_x_len = x_len//factor
    print('hi five',new_x_len)
    # ylimit_value = 0.009
    axs[0, 1].plot(y_disp_up[:new_x_len], label='y_disp_up (Camera)', color='red')
    axs[0, 1].set_xlabel('Time - zoomed (s)')
    axs[0, 1].set_ylabel('Displacement (mm)')
    # axs[0, 1].set_ylim([-ylimit_value, ylimit_value])
    axs[0, 1].set_title('Time domain Plot (Camera)')
    axs[0, 1].set_xticks(np.linspace(0, new_x_len-1, 3))
    axs[0, 1].set_xticklabels(np.arange(0, 3))
    axs[0, 1].grid(True)
    
    Y = np.fft.fft(y_disp_up, N)
    freq_cam = np.fft.fftfreq(N, 1/fs_cam)

    max_freq_cam = freq_cam[np.argmax(np.abs(Y[:N//2]))]
    max_mag_cam = np.max(np.abs(Y[:N//2]))

    # set x-limit to 20 Hz
    freq_mask = freq_cam[:N//2] <= 20
    axs[1, 1].semilogy(freq_cam[:N//2][freq_mask], np.abs(Y[:N//2])[freq_mask], label='FFT (Camera)', color = 'red')
    axs[1, 1].set_xlim([0, 20])

    axs[1, 1].set_title('Frequency Spectrum (Camera)')
    axs[1, 1].set_xlabel('Frequency (Hz)')
    axs[1, 1].set_ylabel('Amplitude')
    axs[1, 1].grid(True)
    axs[1, 1].text(max_freq_cam, max_mag_cam+10**4.2, f'Frequency: {max_freq_cam:.0f} Hz', ha='right', color='red', weight='bold', verticalalignment='center')

    # Add overlapping plots of time and frequency domains from LDV and Camera sources
    axs[0, 2].plot(disp, label='LDV Displacement', color = 'green')
    axs[0, 2].plot(y_disp_up[:new_x_len], label='Camera Displacement', color='red')
    axs[0, 2].set_xlabel('Time (s)')
    axs[0, 2].set_ylabel('Displacement (mm)')
    axs[0, 2].set_title('Time Domain Comparison')
    axs[0, 2].grid(True)
    axs[0, 2].legend()

    axs[1, 2].semilogy(freq[:N//2][freq_mask], np.abs(X[:N//2])[freq_mask], label='FFT (LDV)', color = 'green')
    axs[1, 2].semilogy(freq_cam[:N//2][freq_mask], np.abs(Y[:N//2])[freq_mask], label='FFT (Camera)', color = 'red')
    axs[1, 2].set_title('Frequency Domain Comparison')
    axs[1, 2].set_xlabel('Frequency (Hz)')
    axs[1, 2].set_ylabel('Amplitude')
    axs[1, 2].grid(True)
    axs[1, 2].legend()
    axs[1, 2].set_xlim([0, 20])
    # axs[1, 2].text(max_freq, max_mag+10**4, f'LDV Frequency: {max_freq:.0f} Hz', ha='right', color='green', weight='bold', verticalalignment='center')
    # axs[1, 2].text(max_freq_cam, max_mag_cam+10**4.2, f'Camera Frequency: {max_freq_cam:.0f} Hz', ha='right', color='red', weight='bold', verticalalignment='center')
    
    fig.suptitle(f'3D Displacement Comparison (LDV vs. Camera) - 3Hz signal', fontsize=16)
    plt.savefig(f'3D_Displacement_Comparison_3Hz.png')
    plt.show()
if __name__ == '__main__':
    plot_data('LDV_ARUCO_3Hz.csv', 'camera_1_60s_2023-04-11_22-04-43_3Hz.csv')   