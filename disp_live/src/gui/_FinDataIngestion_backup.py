import rospy
import os
import time
import influxdb_client
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from fiducial_msgs.msg import FiducialTransformArray
token = "CoiF2cx1gda6PLhBrODZ6EIRINU171fhxzmnt9l8_DJr3hf_T8mmzx_ZalQffXvZnUFV1ODn7uNj_mTAf6_OFQ=="
# InfluxDB 2.0 setup
# token = os.environ.get("TETOKEN")
org = "TESOLUTION"
url = "http://localhost:8086"
my_bucket = "BRIDGE"
write_client = InfluxDBClient(url=url, token=token, org=org)
write_api = write_client.write_api(write_options=SYNCHRONOUS)

# Initialize variables to store the first data point for each nuc
first_translation_x_nuc1 = None
first_translation_y_nuc1 = None
first_translation_z_nuc1 = None

first_translation_x_nuc2 = None
first_translation_y_nuc2 = None
first_translation_z_nuc2 = None

first_translation_x_nuc3 = None
first_translation_y_nuc3 = None
first_translation_z_nuc3 = None

first_translation_x_nuc4 = None
first_translation_y_nuc4 = None
first_translation_z_nuc4 = None

first_translation_x_nuc5 = None
first_translation_y_nuc5 = None
first_translation_z_nuc6 = None



def callback_nuc1(data):
    process_data(data, 'nuc1')

def callback_nuc2(data):
    process_data(data, 'nuc2')

def callback_nuc3(data):
    process_data(data, 'nuc3')
    
def callback_nuc4(data):
    process_data(data, 'nuc3')
    
def callback_nuc5(data):
    process_data(data, 'nuc3')

def process_data(data, nuc_name):
    global first_translation_x_nuc1, first_translation_y_nuc1, first_translation_z_nuc1
    global first_translation_x_nuc2, first_translation_y_nuc2, first_translation_z_nuc2
    global first_translation_x_nuc3, first_translation_y_nuc3, first_translation_z_nuc3

    for transform in data.transforms:
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        if nuc_name == 'nuc1':
            # Process data for nuc1
            if first_translation_x_nuc1 is None:
                first_translation_x_nuc1 = translation.x
                first_translation_y_nuc1 = translation.y
                first_translation_z_nuc1 = translation.z

            # Subtract the first data point for nuc1
            normalized_translation_x = translation.x - first_translation_x_nuc1
            normalized_translation_y = translation.y - first_translation_y_nuc1
            normalized_translation_z = translation.z - first_translation_z_nuc1

        elif nuc_name == 'nuc2':
            # Process data for nuc2
            if first_translation_x_nuc2 is None:
                first_translation_x_nuc2 = translation.x
                first_translation_y_nuc2 = translation.y
                first_translation_z_nuc2 = translation.z

            # Subtract the first data point for nuc2
            normalized_translation_x = translation.x - first_translation_x_nuc2
            normalized_translation_y = translation.y - first_translation_y_nuc2
            normalized_translation_z = translation.z - first_translation_z_nuc2

        elif nuc_name == 'nuc3':
            # Process data for nuc3
            if first_translation_x_nuc3 is None:
                first_translation_x_nuc3 = translation.x
                first_translation_y_nuc3 = translation.y
                first_translation_z_nuc3 = translation.z

            # Subtract the first data point for nuc3
            normalized_translation_x = translation.x - first_translation_x_nuc3
            normalized_translation_y = translation.y - first_translation_y_nuc3
            normalized_translation_z = translation.z - first_translation_z_nuc3

        # Save normalized data to InfluxDB
        save_to_influxdb(nuc_name, normalized_translation_x, normalized_translation_y, normalized_translation_z, rotation)

def save_to_influxdb(nuc_name, translation_x, translation_y, translation_z, rotation):
    point = Point("fiducial_transforms") \
        .tag("nuc", nuc_name) \
        .field("translation_x", translation_x) \
        .field("translation_y", translation_y) \
        .field("translation_z", translation_z) \
        .field("rotation_x", rotation.x) \
        .field("rotation_y", rotation.y) \
        .field("rotation_z", rotation.z) \
        .field("rotation_w", rotation.w) \
        .time(time.time_ns(), WritePrecision.NS)
    write_api.write(bucket=my_bucket, record=point)

if __name__ == '__main__':
    rospy.init_node('data_processor', anonymous=True)
    rospy.Subscriber("/sony_1/fiducial_transforms", FiducialTransformArray, callback_nuc1)
    # rospy.Subscriber("/camera_2/fiducial_transforms", FiducialTransformArray, callback_nuc2)
    # rospy.Subscriber("/camera_3/fiducial_transforms", FiducialTransformArray, callback_nuc3)
    try:
        rospy.spin()
    finally:
        write_api.close()
        write_client.close()
