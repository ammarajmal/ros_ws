#!/usr/bin/env python3
# themes = {'blue': ("#3B8ED0", "#1F6AA5"),
#           'green': ("#2CC985", "#2FA572"),
#           'dark-blue': ("#3a7ebf", "#1f538d")
# }

# color_select = list(themes.keys())[2]  # selection: (0: 'blue') (1: 'green') (2: 'dark-blue')


# print(color_select)


# filename_ = '/home/ammar/ros_ws/src/gige_cam_driver/bagfiles/cameras_12_10s_2023-05-04_12-17-23.bag'
# cam_num = '12'
# for cam in cam_num:
#     camera_name = 'camera_' + cam
#     print(camera_name)
#     new_filename_ = filename_.replace('.bag', f'_{cam}.bag')
#     print(new_filename_)
import os
filename_ = '/home/ammar/ros_ws/src/gige_cam_driver/csvfiles/cameras_12_5s_2023-05-22_18-34-56_cam2.csv'
file = os.path.basename(filename_)
# # find the part before first underscore
# if file.split('_')[0] == 'cameras':
#     print('multiple cameras')
# else:
#     print('single camera')

# # separate the number in the string (file.split('_')[1]) and save in separate variable
# cam_num = (file.split('_')[1])
# print(cam_num, len(cam_num))



# cameraData = pd.read_csv(camera_file)
# print('in plotting function - camera_file name:',camera_file)
camera_file = file

record_cam_number  = camera_file.split('_')[1]
record_duration = camera_file.split('_')[2][:-1]

# record_date = camera_file.split('_')[3]
# record_time = camera_file.split('_')[4]
# record_hz = camera_file.split('_')[5].split('Hz')[0]

print("Recoded Details:")
print("Camera Number: ", record_cam_number)
print("Record Duration: ", record_duration, 's')
# print("Record Date: ", record_date)
# print("Record Time: ", record_time)
# print("Record Frequency: ", record_hz, 'Hz')
