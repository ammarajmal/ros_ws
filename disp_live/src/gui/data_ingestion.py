import rospy
import os
import time
import influxdb_client
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from fiducial_msgs.msg import FiducialTransformArray

# InfluxDB 2.0 setup
token = os.environ.get("INFLUXDB_TOKEN")
org = "Chung-Ang University"
url = "http://localhost:8086"
write_client = InfluxDBClient(url=url, token=token, org=org)
write_api = write_client.write_api(write_options=SYNCHRONOUS)

# Initialize variables to store the last timestamp for each nuc
last_timestamp_nuc1 = None
last_timestamp_nuc2 = None
last_timestamp_nuc3 = None

# Initialize variables to store the frequency of data points for each nuc
data_frequency_nuc1 = 0
data_frequency_nuc2 = 0
data_frequency_nuc3 = 0

def callback_nuc1(data):
    process_data(data, 'nuc1')

def callback_nuc2(data):
    process_data(data, 'nuc2')

def callback_nuc3(data):
    process_data(data, 'nuc3')

def process_data(data, nuc_name):
    global last_timestamp_nuc1, last_timestamp_nuc2, last_timestamp_nuc3
    global data_frequency_nuc1, data_frequency_nuc2, data_frequency_nuc3

    for transform in data.transforms:
        timestamp = rospy.Time.now().to_sec()  # Get the current timestamp

        if nuc_name == 'nuc1':
            if last_timestamp_nuc1 is not None:
                # Calculate time difference between current and last data point for nuc1
                time_difference = timestamp - last_timestamp_nuc1

                # Calculate frequency for nuc1 (data points per second)
                data_frequency_nuc1 = 1.0 / time_difference

            last_timestamp_nuc1 = timestamp

        elif nuc_name == 'nuc2':
            if last_timestamp_nuc2 is not None:
                # Calculate time difference between current and last data point for nuc2
                time_difference = timestamp - last_timestamp_nuc2

                # Calculate frequency for nuc2 (data points per second)
                data_frequency_nuc2 = 1.0 / time_difference

            last_timestamp_nuc2 = timestamp

        elif nuc_name == 'nuc3':
            if last_timestamp_nuc3 is not None:
                # Calculate time difference between current and last data point for nuc3
                time_difference = timestamp - last_timestamp_nuc3

                # Calculate frequency for nuc3 (data points per second)
                data_frequency_nuc3 = 1.0 / time_difference

            last_timestamp_nuc3 = timestamp

        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Save normalized data to InfluxDB
        save_to_influxdb(nuc_name, translation, rotation)

def save_to_influxdb(nuc_name, translation, rotation):
    point = Point("fiducial_transforms") \
        .tag("nuc", nuc_name) \
        .field("translation_x", translation.x) \
        .field("translation_y", translation.y) \
        .field("translation_z", translation.z) \
        .field("rotation_x", rotation.x) \
        .field("rotation_y", rotation.y) \
        .field("rotation_z", rotation.z) \
        .field("rotation_w", rotation.w) \
        .time(time.time_ns(), WritePrecision.NS)
    write_api.write(bucket="SITL", record=point)

if __name__ == '__main__':
    rospy.init_node('data_processor', anonymous=True)
    rospy.Subscriber("/nuc1/fiducial_transforms", FiducialTransformArray, callback_nuc1)
    rospy.Subscriber("/nuc2/fiducial_transforms", FiducialTransformArray, callback_nuc2)
    rospy.Subscriber("/nuc3/fiducial_transforms", FiducialTransformArray, callback_nuc3)
    
    try:
        rospy.spin()
    finally:
        write_api.close()
        write_client.close()

    # Print the calculated data frequencies for each nuc
    print(f"Data Frequency (nuc1): {data_frequency_nuc1} Hz")
    print(f"Data Frequency (nuc2): {data_frequency_nuc2} Hz")
    print(f"Data Frequency (nuc3): {data_frequency_nuc3} Hz")
