#!/usr/bin/env python3
name ='/home/agcam/ros_ws/src/gige_cam_driver/bagfiles/camera_1_2s_2023-03-27_11-13-00.bag'
def get_camera_name(string_cam):
    try:
        camera_name =  "_".join(string_cam.split('/')[-1].split('_')[:2])
        complete_filename = string_cam.split('/')[-1].split('.')[0]
        return [camera_name, complete_filename]
    except:
        return None

camera_name = get_camera_name(name)[0]
just_filename = get_camera_name(name)[1]

print(camera_name)
print(just_filename)