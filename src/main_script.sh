#!/bin/bash

# # Run the launch command
# roslaunch gige_cam_driver cameras.launch
echo "Please select the camera you want to calibrate: (1. Camera 1, 2. Camera 2, 3. Camera 3))"

# Provide the user with options to choose from
options=("Camera 1" "Camera 2" "Camera 3")

# Use the select statement to display the options
select opt in "${options[@]}"
do
    case $opt in
        "Camera 1")
            # Run camera calibration command for Camera 1
            rosrun camera_calibration cameracalibrator.py --size 6x5 --square 0.025 --k-coefficients=2 --fix-principal-point i --fix-aspect-ratio image:=/camera_1/image_raw camera:=/camera_1
            break
            ;;
        "Camera 2")
            # Run camera calibration command for Camera 2
            rosrun camera_calibration cameracalibrator.py --size 6x5 --square 0.025 --k-coefficients=2 --fix-principal-point i --fix-aspect-ratio image:=/camera_2/image_raw camera:=/camera_2
            break
            ;;
        "Camera 3")
            # Run camera calibration command for Camera 3
            rosrun camera_calibration cameracalibrator.py --size 6x5 --square 0.025 --k-coefficients=2 --fix-principal-point i --fix-aspect-ratio image:=/camera_3/image_raw camera:=/camera_3
            break
            ;;
        *) echo "Invalid option selected";;
    esac
done

# Ask the user if they want to save the camera data
read -p "Do you want to save the camera data for offline use (y/n)? " save_data

if [ "$save_data" == "y" ]; then
    # Ask the user if they want to save the data for all cameras in a single file
    read -p "Do you want to save the data for all cameras in a single file (y/n)? " save_all_cams
    
    if [ "$save_all_cams" == "y" ]; then
        # Save the data for all cameras in a single file
        roslaunch gige_cam_driver bag_cams.launch
    else
        # Ask the user which camera's data they want to save
        read -p "Which camera's data do you want to save (1/2/3)? " cam_number
        
        case $cam_number in
            1)
                # Save Camera 1 data
                roslaunch gige_cam_driver bag_cam1.launch
                ;;
            2)
                # Save Camera 2 data
                roslaunch gige_cam_driver bag_cam2.launch
                ;;
            3)
                # Save Camera 3 data
                roslaunch gige_cam_driver bag_cam3.launch
                ;;
            *)
                echo "Invalid camera number selected."
                ;;
        esac
    fi
fi

echo "The process finished successfully."
