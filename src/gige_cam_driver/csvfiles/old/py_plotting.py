#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np
from scipy.fftpack import fft

def py_plot(file):
    # Read data from the csv file and plot it
    dataframe = pd.read_csv(file)
    
    # Print the head info of the csv file
    # print(dataframe.head())
    displacement = dataframe['field.transforms0.transform.translation.x']
    print('Type of the displacement array: ', type(displacement))
    print('Size of the displacement array: ', displacement.size)
    
    # Calculate the time array
    time = np.linspace(0, displacement.size / 2, displacement.size)  # Assuming 2Hz sampling rate
    
    # Perform a Fast Fourier Transform (FFT) on the displacement data
    spectrum = fft(displacement.to_numpy())  # Convert the pandas Series to a NumPy array
    freqs = np.fft.fftfreq(displacement.size, 1 / 2)  # Assuming 2Hz sampling rate
    
    # Now using subplots, plot time domain and frequency domain plot of displacement
    fig, axes = plt.subplots(2, 1)
    
    # Time domain plot
    axes[0].plot(time, displacement.to_numpy())  # Convert the pandas Series to a NumPy array
    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Displacement')
    axes[0].set_title('Time Domain Plot of Displacement')
    
    # Frequency domain plot
    axes[1].plot(freqs[:displacement.size // 2], np.abs(spectrum[:displacement.size // 2]))
    axes[1].set_xlabel('Frequency (Hz)')
    axes[1].set_ylabel('Magnitude')
    axes[1].set_title('Frequency Domain Plot of Displacement')
    
    # Adjust layout and display the plots
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
    # Read from the csv file passed as file
    csv_file = 'camera_1_30s_2023-04-24_11-16-23_6Hz.csv'
    path= '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/'
    csv_file_path = os.path.join(path, csv_file)
    py_plot(csv_file_path)
