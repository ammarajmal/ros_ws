#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import matplotlib.pyplot as plt

# Global variables for storing the received waveform values
waveform_values = []
timestamps = []

def callback(data):
    global waveform_values
    global timestamps

    waveform_values.append(data.data)
    timestamps.append(rospy.Time.now().to_sec())  # Record the timestamp of each received waveform value

def subscriber_node():
    rospy.init_node('subscriber_node', anonymous=True)
    rospy.Subscriber('sine_py', Float64, callback)

    # Initialize the plot
    fig, ax = plt.subplots()
    ax.set_xlabel('Time')
    ax.set_ylabel('Waveform Value')
    ax.set_title('Real-Time Sinusoidal Waveform')

    plt.show(block=False)  # Display the plot window without blocking the program execution

    rate = rospy.Rate(100)  # Update plot at a rate of 10 Hz

    while not rospy.is_shutdown():
        # Update the plot with the received waveform values
        ax.plot(timestamps, waveform_values, 'b-')

        ax.relim()
        ax.autoscale_view(True, True, True)
        plt.draw()

        plt.pause(0.001)  # Pause for a short duration to allow the plot to update

        rate.sleep()

if __name__ == '__main__':
    subscriber_node()
