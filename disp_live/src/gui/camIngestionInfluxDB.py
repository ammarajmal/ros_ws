import rospy
import os
import time
import json
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from sensor_msgs.msg import CameraInfo

# InfluxDB 2.0 setup
token = os.environ.get("INFLUXDB_OMEN_CUR_TOKEN")
org = "CAUSITL"
url = "http://localhost:8086"
write_client = InfluxDBClient(url=url, token=token, org=org)
write_api = write_client.write_api(write_options=SYNCHRONOUS)

def save_to_influxdb(camera_info):
    # Convert the 'D', 'K', 'P', and 'R' fields from tuples to JSON strings
    D_json = json.dumps(camera_info.D)
    K_json = json.dumps(camera_info.K)
    P_json = json.dumps(camera_info.P)
    R_json = json.dumps(camera_info.R)

    point = Point("camera_info") \
        .tag("camera", camera_info.header.frame_id) \
        .field("height", camera_info.height) \
        .field("width", camera_info.width) \
        .field("distortion_model", camera_info.distortion_model) \
        .field("D", D_json) \
        .field("K", K_json) \
        .field("R", R_json) \
        .field("P", P_json) \
        .field("binning_x", camera_info.binning_x) \
        .field("binning_y", camera_info.binning_y) \
        .field("x_offset", camera_info.roi.x_offset) \
        .field("y_offset", camera_info.roi.y_offset) \
        .field("roi_height", camera_info.roi.height) \
        .field("roi_width", camera_info.roi.width) \
        .field("do_rectify", camera_info.roi.do_rectify) \
        .time(time.time_ns(), WritePrecision.NS)

    write_api.write(bucket="CAUSITLOMEN", record=point)

def callback(data):
    save_to_influxdb(data)

if __name__ == '__main__':
    rospy.init_node('data_processor', anonymous=True)
    topic_name = "/camera_0/camera_info"
    
    rospy.Subscriber(topic_name, CameraInfo, callback)

    try:
        rospy.spin()
    finally:
        write_api.close()
        write_client.close()
