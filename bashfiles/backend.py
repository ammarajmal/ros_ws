#!/usr/bin/env python3
""" backend module for definitions of the backend functions"""

import rospkg # For getting the path of the ros packages
import roslaunch # For launching the ros nodes

cam_launch = rospkg.RosPack().get_path('gige_cam_driver')
view_cam_launch = rospkg.RosPack().get_path('gige_cam_driver')
print(cam_launch)


uuid_ = roslaunch.rlutil.get_or_generate_uuid(None, False)
# roslaunch.configure_logging(uuid_)
BOARD_SIZE = '8x6'
SQUARE_SIZE = '0.108'
CAMERA_SELECT = 2
# CAMERA_ACTIVE = 'camera_2'
# CAMERA_ACTIVE = 'camera_3'

CAMERA_STATE_1 = CAMERA_STATE_2 = CAMERA_STATE_1 = None
CAMERA_START_1, CAMERA_START_2, CAMERA_START_3 = 'Camera 1', 'Camera 2', 'Camera 3'
CALIIB_START_1, CALIB_START_2, CALIB_START_3 = 'Calibrate 1', 'Calibrate 2', 'Calibrate 3'

cam_ = [{'camera_name': 'camera_1',
         'device_id': 0,
         'calibration_file': 'cam1',
         'name': 'Camera 1',
         'button': CAMERA_START_1,
         'calibrate_button': CALIIB_START_1},
        {'camera_name': 'camera_2',
         'device_id': 1,
         'calibration_file': 'cam2',
         'name': 'Camera 2',
         'button': CAMERA_START_2,
         'calibrate_button': CALIB_START_2},
        {'camera_name': 'camera_3',
         'device_id': 2,
         'calibration_file': 'cam3',
         'name': 'Camera 3',
         'button': CAMERA_START_3,
        'calibrate_button': CALIB_START_3}
        ]
def camera_param(camera_num):
    """_summary_
    """
    if camera_num < 1 or camera_num > 3:
        raise ValueError("Camera number must be between 1 and 3")
    camera_sel = cam_[camera_num - 1]
    print(camera_sel)

def camera_event():
    """_summary_
    """
    print(cam_launch)
    print(view_cam_launch)
def camera_calib_event():
    """_summary_
    """
    print("Camera Calibration button clicked")
def destroy_routine():
    """_summary_
    """
    # global process
    # print("Exit button clicked")
    # if process is not None:
    #     process.terminate()
    # # Exit the GUI here
    # if process is not None:
    #     return_code = process.poll()
    #     if return_code is None:
    #         print("Process is still running")
    #     else:
    #         print(f"Process has terminated with exit code {return_code}")
    # else:
    #     print("Process has not been started yet")
    #     # Exit the GUI here
    #     exit(0)

if __name__ == "__main__":
    camera_param(1)
    camera_param(2)
    camera_param(3)
    # camera_param(4)
    # camera_param(0)
    # start_camera_callback()
    # start_camera_calibration_callback()
    # destroy_routine()
    