#!/bin/bash


echo 
echo "3-D Displacement Calculation:"
echo 
while true; do
    echo "Please select an option:"
    echo "1. Start Camera Calibration"
    echo "2. Save Camera data for offline processing"
    echo "3. Load offline saved data and perform displacement calculation"
    echo "4. Real-time displacement calculation"
    echo "5. Exit"
    read -p "Enter your choice [1-5]: " choice
    case $choice in
        1)
            echo "Starting Camera Calibration..."
            echo 
            # Camera calibration code
            echo "Please select the camera you want to calibrate: (1. Camera 1, 2. Camera 2, 3. Camera 3))"

            # Provide the user with options to choose from
            options=("Camera 1" "Camera 2" "Camera 3")

            # Use the select statement to display the options
            select opt in "${options[@]}"
            do
                case $opt in
                    "Camera 1")
                        # Run camera calibration command for Camera 1
                        roslaunch gige_cam_driver calib1.launch
                        echo "Camera 1 calibration finished."
                        break
                        ;;
                    "Camera 2")
                        # Run camera calibration command for Camera 2
                        roslaunch gige_cam_driver calib2.launch
                        echo "Camera 2 calibration finished."
                        break
                        ;;
                    "Camera 3")
                        # Run camera calibration command for Camera 3
                        roslaunch gige_cam_driver calib3.launch
                        echo "Camera 3 calibration finished."
                        break
                        ;;
                    *) echo "Invalid option selected";;
                esac
            done

            ;;
        2)
            echo "Saving Camera data for offline processing..."
            # Save Camera data code

            # Ask the user if they want to save the data for all cameras in a single file
            read -p "Do you want to save the data for all cameras in a single file (y/n)? " save_all_cams
            
            if [ "$save_all_cams" == "y" ]; then
                # Save the data for all cameras in a single file
                gnome-terminal -- roslaunch gige_cam_driver allcams.launch & cameras_pid=$!
                sleep 3
                gnome-terminal -- roslaunch gige_cam_driver allbags.launch & record_pid=$!
                sleep 20
                # Wait for the rosbag recording node to exit
                wait $record_pid

                # Kill the camera nodes if the rosbag recording node has exited
                if ! ps -p $cameras_pid > /dev/null; then
                    echo "Killing camera 1 node..."
                    rosnode kill /camera_1
                    rosnode kill /camera_2
                    rosnode kill /camera_3
                fi

            else
                # Ask the user which camera's data they want to save
                read -p "Which camera's data do you want to save (1/2/3)? " cam_number
                
                case $cam_number in
                    1)
                        # Save Camera 1 data
                        gnome-terminal -- roslaunch gige_cam_driver cam1.launch & cam1_pid=$!
                        sleep 3
                        gnome-terminal -- roslaunch gige_cam_driver bag1.launch & record_pid=$!
                        sleep 5
                        # Wait for the rosbag recording node to exit
                        wait $record_pid
                        if ! ps -p $cam1_pid > /dev/null; then
                            echo "Killing camera 1 node..."
                            rosnode kill /camera_1
                        fi
                        ;;
                    2)
                        # Save Camera 2 data
                        gnome-terminal -- roslaunch gige_cam_driver cam2.launch & cam2_pid=$!
                        sleep 3
                        gnome-terminal -- roslaunch gige_cam_driver bag2.launch & record_pid=$!
                        sleep 5
                        # Wait for the rosbag recording node to exit
                        wait $record_pid
                        if ! ps -p $cam2_pid > /dev/null; then
                            echo "Killing camera 2 node..."
                            rosnode kill /camera_2
                        fi
                        ;;
                    3)
                        # Save Camera 3 data
                        gnome-terminal -- roslaunch gige_cam_driver cam3.launch & cam3_pid=$!
                        sleep 3
                        gnome-terminal -- roslaunch gige_cam_driver bag3.launch & record_pid=$!
                        sleep 5
                        # Wait for the rosbag recording node to exit
                        wait $record_pid
                        if ! ps -p $cam3_pid > /dev/null; then
                            echo "Killing camera 3 node..."
                            rosnode kill /camera_3
                        fi
                        
                        ;;
                    *)
                        echo "Invalid camera number selected."
                        ;;
                esac
            fi

            ;;
        3)
            echo "Loading offline saved data and performing displacement calculation..."
            # Loading and calculation code
            read -p "which Camera data do you want to load (1/2/3)? " cam_number
            case $cam_number in
                1)
                    # Load Camera 1 data by launching readbag1.launch file from gige_cam_driver package into a new terminal
                    gnome-terminal -- roslaunch gige_cam_driver readbag1.launch & rosbag_pid=$!
                    sleep 1
                    # Load aruco_detect node from aruco_detect package into a new terminal 
                    gnome-terminal -- roslaunch aruco_detect cam1.launch & aruco_pid=$!
                    sleep 2
                    # save the topic /cam1_marker/fiducial_transforms into a bag file into a new terminal with name detect_cam1.bag 
                    # gnome-terminal rosbag record --output-name=detect_cam1.bag /cam1_marker/fiducial_transforms
                    rosbag record /cam1_marker/fiducial_transforms -o cam1.bag
                    # when readbag1.launch exits, kill the aruco_detect node and the rosbag record node
                    # if ! ps -p $rosbag_pid > /dev/null; then
                    #     echo "Killing aruco_detect node..."
                    #     rosnode kill -a
                    #     echo "Killing rosbag record node..."
                    #     rosnode kill /rosbag_record
                    # fi

                    
                    # rostopic echo /cam1_marker/fiducial_transforms > cam1_marker.csv

                    







                    # Now store the translation x, y and z  from rostopic echo /cam1_marker/fiducial_transforms into a csv file in the same directory
                    # rostopic echo /cam1_marker/fiducial_transforms > cam1_data.csv &
                    # rostopic echo /cam1_marker/fiducial_transforms | grep -e 'header:\|fiducial_id:\|translation:' | sed 's/.*header: { secs: \([0-9]*\), nsecs: \([0-9]*\), frame_id: \"\(.*\)\" }.*/\1,\2,\3/g' | sed 's/.*fiducial_id: \([0-9]*\).*/\1/g' | sed 's/.*translation: { x: \([-0-9.]*\), y: \([-0-9.]*\), z: \([-0-9.]*\).*/\1,\2,\3/g' > cam1_data.csv &


rostopic 
                    # rostopic echo /cam1_marker/fiducial_transforms > cam1_data.csv &
                    # sleep 1
                    # Wait for the rosbag process to finish
                    wait $rosbag_pid
                    # Terminate the aruco_detect process
                    kill $aruco_pid
                    # Kill all the terminal windows
                    
                    pkill gnome-terminal
                    # Display a message that the work is done
                    echo "Work is done!"


                    ;;

                2)
                    # Load Camera 2 data
                    ;;
                3)
                    # Load Camera 3 data
                    ;;
                *)
                    echo "Invalid camera number selected."
                    ;;
            esac

          
            ;;
        4)
            echo "Performing real-time displacement calculation..."
            echo 
            # Real-time calculation code here
            roslaunch gige_cam_driver allcams.launch & roslaunch aruco_detect cams.launch
            ;;
        5)
            echo "Exiting..."
            exit 0
            ;;
        *)
            echo "Invalid option. Please select an option from 1 to 5."
            echo 
            ;;
    esac
done


