import rospy
import os
import time
import influxdb_client
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from fiducial_msgs.msg import FiducialTransformArray

# InfluxDB 2.0 setup
token = os.environ.get("INFLUXDB_OMEN_CUR_TOKEN")
org = "CAUSITL"
url = "http://localhost:8086"
write_client = InfluxDBClient(url=url, token=token, org=org)
write_api = write_client.write_api(write_options=SYNCHRONOUS)

# Initialize variables to store the first data point for each camera
camera_data = {}

def initialize_camera_data(camera_names):
    for camera_name in camera_names:
        camera_data[camera_name] = {
            'first_translation_x': None,
            'first_translation_y': None,
            'first_translation_z': None
        }

def callback(data, camera_name):
    process_data(data, camera_name)

def process_data(data, camera_name):
    if camera_name not in camera_data:
        rospy.logwarn(f"Received data for unknown camera: {camera_name}")
        return

    first_data = camera_data[camera_name]

    for transform in data.transforms:
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        if first_data['first_translation_x'] is None:
            first_data['first_translation_x'] = translation.x
            first_data['first_translation_y'] = translation.y
            first_data['first_translation_z'] = translation.z

        # Subtract the first data point
        normalized_translation_x = translation.x - first_data['first_translation_x']
        normalized_translation_y = translation.y - first_data['first_translation_y']
        normalized_translation_z = translation.z - first_data['first_translation_z']

        # Save normalized data to InfluxDB
        save_to_influxdb(camera_name, normalized_translation_x, normalized_translation_y, normalized_translation_z, rotation)

def save_to_influxdb(camera_name, translation_x, translation_y, translation_z, rotation):
    point = Point("fiducial_transforms") \
        .tag("camera", camera_name) \
        .field("translation_x", translation_x) \
        .field("translation_y", translation_y) \
        .field("translation_z", translation_z) \
        .field("rotation_x", rotation.x) \
        .field("rotation_y", rotation.y) \
        .field("rotation_z", rotation.z) \
        .field("rotation_w", rotation.w) \
        .time(time.time_ns(), WritePrecision.NS)

    write_api.write(bucket="CAUSITLOMEN", record=point)

if __name__ == '__main__':
    rospy.init_node('data_processor', anonymous=True)

    # Define the camera topics and their corresponding callback functions
    camera_topics = {
        "/camera_1/fiducial_transforms": "camera_1",
        "/camera_2/fiducial_transforms": "camera_2",
        "/camera_3/fiducial_transforms": "camera_3",
        "/camera_4/fiducial_transforms": "camera_4",
        "/camera_5/fiducial_transforms": "camera_5",
    }

    initialize_camera_data(camera_topics.values())

    for topic, camera_name in camera_topics.items():
        rospy.Subscriber(topic, FiducialTransformArray, callback, camera_name)

    try:
        rospy.spin()
    finally:
        write_api.close()
        write_client.close()
