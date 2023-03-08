#!/usr/bin/env python

import os
import time

def start_calibration():
    """function to start camera calibration
    
    Raises:
        ValueError: if the user enters an invalid option for camera selection
    """
    print('---------------------------------')
    print("  Starting camera calibration    ")
    print('---------------------------------')
    print("Please select the camera you want to calibrate:")
    # Provide the user with options to choose from
    options = ["Camera-1", "Camera-2", "Camera-3"]
    # Use the select statement to display the options
    while True:
        for i, opt in enumerate(options):
            print(f"{i+1}. {opt}")
        cam_num = input("\nCamera No.: ")
        try:
            cam_num = int(cam_num)
            if cam_num < 1 or cam_num > len(options):
                raise ValueError
            break
        except ValueError:
            print("\nInvalid option selected. Please select among [1-3]:")
    if cam_num == 1:
        # Run camera calibration command for Camera-1
        print('---------------------------------------')
        print("  Starting calibration for Camera-1    ")
        print('---------------------------------------')
        time.sleep(0.5)
        os.system("roslaunch gige_cam_driver calib1.launch")
        print("Camera 1 calibration finished.\n")
    elif cam_num == 2:
        # Run camera calibration command for Camera-2
        print('---------------------------------------')
        print("  Starting calibration for Camera-2    ")
        print('---------------------------------------')
        time.sleep(0.5)
        os.system("roslaunch gige_cam_driver calib2.launch")
        print("Camera 2 calibration finished.\n")
    elif cam_num == 3:
        # Run camera calibration command for Camera-3
        print('---------------------------------------')
        print("  Starting calibration for Camera-3    ")
        print('---------------------------------------')
        time.sleep(0.5)
        os.system("roslaunch gige_cam_driver calib3.launch")
        print("Camera 3 calibration finished.\n")

def saving_data_offline():
        """function to save camera data for post processing
        """
        print('--------------------------------------------')
        print("  Saving camera data for post processing    ")
        print
        ('--------------------------------------------')
        save_option = input("Do you want to save all cameras data in a single file:")
            # Provide the user with options to choose from
        options = ["y / yes", "n / no"]


    


while True:
    print('====================================')
    print("     3-D Displacement Calculation   ")
    print('====================================')
    print("     Please select an option:")
    print("1. Start Camera Calibration")
    print("2. Save Camera data for offline processing")
    print("3. Load offline saved data and perform displacement calculation")
    print("4. Real-time displacement calculation")
    print("5. Exit\n")
    choice = int(input("Enter your choice [1-5]: "))
    
    if choice == 1:
        start_calibration()
    elif choice == 2:
        saving_data_offline()
        os.system("save_camera_data.sh")
    elif choice == 3:
        os.system("load_offline_data.sh")
    elif choice == 4:
        os.system("real_time_displacement.sh")
    elif choice == 5:
        break
    else:
        print("Invalid choice. Please enter a number between 1 and 5.")


    # continu1e
    
        
    
    
    
    # if choice == "1":
    #     os.system("start_camera_calibration.sh")
    # elif choice == "2":
    #     os.system("save_camera_data.sh")
    # elif choice == "3":
    #     os.system("load_offline_data.sh")
    # elif choice == "4":
    #     os.system("real_time_displacement.sh")
    # elif choice == "5":
    #     break
    # else:
    #     print("Invalid choice. Please enter a number between 1 and 5.")
