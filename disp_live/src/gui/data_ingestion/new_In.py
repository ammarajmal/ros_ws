import rospy
import os
import time
import influxdb_client
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from geometry_msgs.msg import PoseStamped

# InfluxDB 2.0 setup
token = os.environ.get("NEWTOKEN")
org = "CAUSITL"
url = "http://localhost:8086"
write_client = InfluxDBClient(url=url, token=token, org=org)
write_api = write_client.write_api(write_options=SYNCHRONOUS)

# Initialize variables to store the first data point for camera 0
first_translation_x = None
first_translation_y = None
first_translation_z = None

def callback(data):
    process_data(data)

def process_data(data):
    global first_translation_x, first_translation_y, first_translation_z

    translation = data.pose.position
    rotation = data.pose.orientation

    if first_translation_x is None:
        first_translation_x = translation.x
        first_translation_y = translation.y
        first_translation_z = translation.z

    # Subtract the first data point
    normalized_translation_x = translation.x - first_translation_x
    normalized_translation_y = translation.y - first_translation_y
    normalized_translation_z = translation.z - first_translation_z

    # Save normalized data to InfluxDB
    save_to_influxdb(normalized_translation_x, normalized_translation_y, normalized_translation_z, rotation)

def save_to_influxdb(translation_x, translation_y, translation_z, rotation):
    point = Point("ammar") \
        .tag("camera", "camera_0") \
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

    rospy.Subscriber("/sony_1/fiducial_transforms", PoseStamped, callback)

    try:
        rospy.spin()
    finally:
        write_api.close()
        write_client.close()
