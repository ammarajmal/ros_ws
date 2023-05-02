#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Float32
import matplotlib
matplotlib.use('TkAgg')  # Set the backend to TkAgg
import matplotlib.pyplot as plt

# Global variables for the buffer and the sampling rate
buffer = []
fs = 1000

# Callback function for the ROS subscriber
def callback(msg):
    # print(msg.data)
    global buffer
    # Append the new data to the buffer
    buffer.append(msg.data)
    print('BUFFER:', buffer)

# Function to update the plot
def update_plot():
    global buffer

    # Apply the FFT to the buffer
    N = len(buffer)
    X = np.fft.fft(buffer, N)
    freq = np.fft.fftfreq(N, 1/fs)

    # Plot the frequency spectrum
    plt.clf()
    plt.semilogy(freq[:N//2][freq[:N//2] <= 100], np.abs(X[:N//2])[freq[:N//2] <= 100], color='green')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Magnitude')
    plt.title('Frequency Spectrum')
    plt.ylim([10**(-4), 10**1])
    plt.xlim([0, 100])
    plt.grid(True)
    plt.show(block=False)

    # Clear the buffer
    buffer.clear()

    # Pause the plot window for a short period of time
    plt.pause(0.01)

    # Close the plot window
    plt.close()

if __name__ == '__main__':
    # Initialize the ROS node and the subscriber
    rospy.init_node('sine_frequency_plot')
    sub = rospy.Subscriber('sine', Float32, callback)

    # Initialize the plot window
    plt.ion()
    plt.show()

    # Update the plot on key press
    while not rospy.is_shutdown():
        input("Press Enter to update the plot...")
        update_plot()
