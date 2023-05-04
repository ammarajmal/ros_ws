#!/usr/bin/env python3
# themes = {'blue': ("#3B8ED0", "#1F6AA5"),
#           'green': ("#2CC985", "#2FA572"),
#           'dark-blue': ("#3a7ebf", "#1f538d")
# }

# color_select = list(themes.keys())[2]  # selection: (0: 'blue') (1: 'green') (2: 'dark-blue')


# print(color_select)


filename_ = '/home/ammar/ros_ws/src/gige_cam_driver/bagfiles/cameras_12_10s_2023-05-04_12-17-23.bag'
cam_num = '12'
for cam in cam_num:
    camera_name = 'camera_' + cam
    print(camera_name)
    new_filename_ = filename_.replace('.bag', f'_{cam}.bag')
    print(new_filename_)


                   