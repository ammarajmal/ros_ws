#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
import math
import time
import os
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS

# InfluxDB 2.0 setup
bucket = "CAUSITLOMEN"
token = os.environ.get("INFLUXDB_OMEN_CUR_TOKEN")
org = "CAUSITL"
url = "http://localhost:8086"
write_client = InfluxDBClient(url=url, token=token, org=org)
write_api = write_client.write_api(write_options=SYNCHRONOUS)

def publisher_node():
    rospy.init_node('publisher_node', anonymous=True)
    pub = rospy.Publisher('sine_py', Float64, queue_size=10)

    amplitude = 1.0  # Amplitude of the waveform
    frequency = 1.0  # Frequency of the waveform in Hz
    time = 0.0      # Initial time
    sampling_frequency = 100.0  # Sampling frequency of the waveform in Hz
    rate = rospy.Rate(sampling_frequency)  # Publish at a rate of 10 Hz

    while not rospy.is_shutdown():
        # Calculate the value of the waveform using a sinusoidal function
        waveform_value = amplitude * math.sin(2 * math.pi * frequency * time)

        # Create a Float64 message and publish the waveform value
        message = Float64()
        message.data = waveform_value
        pub.publish(message)

        # Save normalized data to InfluxDB
        save_to_influxdb(waveform_value)

        # Increment the time
        time += 1.0 / sampling_frequency  # 10 Hz publishing rate, so increment time by 1/10th of a second

        rate.sleep()

def save_to_influxdb(waveform_value):
    point = Point("sine_waveform") \
        .field("value", waveform_value) \
        .time(int(time.time() * 1e9), WritePrecision.NS)  # Convert to nanoseconds

    write_api.write(bucket=bucket, record=point)

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
    finally:
        write_api.close()
        write_client.close()
