#!/usr/bin/env python3
import roslaunch
import rospkg
import rospy
import time
import subprocess
import pandas as pd
import matplotlib.pyplot as plt
import os
import datetime


class LaunchHandle(object):
    def __init__(self):
        rospy.init_node('launch_handle', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.package = 'gige_cam_driver'
        
        # ********************************************************************************
        # Path management for Launch files
        # ********************************************************************************
        
        self.launch_path = rospkg.RosPack().get_path('gige_cam_driver') + '/launch/'
        self.detect_launch_path = rospkg.RosPack().get_path('aruco_detect') + '/launch/'
        self.bagfile_path = self.launch_path.replace("launch/", "bagfiles/")

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)

        self.record_bag_launch   = f"{self.launch_path}recordbag.launch"
        self.read_bag_launch  = f"{self.launch_path}readbag.launch"
        self.cam_launch    = f"{self.launch_path}cam.launch"
        self.detect_launch = f"{self.detect_launch_path}detect.launch"

        
        # ********************************************************************************
        # Camera Node code for multiple cameras
        # ********************************************************************************

        # running camera node for all cameras
        cam_dict = {'cam': ['camera_1', 'camera_2', 'camera_3'], 'device_id': ['0', '1', '2'], 'calib_file': ['cam1', 'cam2', 'cam3']}
        for i in range(len(cam_dict['cam'])):
            cli_args = [
                self.cam_launch,
                f"cam:={cam_dict['cam'][i]}",
                f"device_id:={cam_dict['device_id'][i]}",
                f"calib_file:={cam_dict['calib_file'][i]}"
            ]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
            node_name = f"cam{i+1}_driver"
            setattr(self, node_name, roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file))
        # ********************************************************************************
        
        
        
        
        # ********************************************************************************
        # Marker Detection code for multiple cameras
        # ********************************************************************************
        cli_args = [self.detect_launch, 'camera:=camera_1']
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        self.marker_detect_camera_1 = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        
        
        cli_args = [self.detect_launch, 'camera:=camera_2']
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        self.marker_detect_camera_2 = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        
        
        cli_args = [self.detect_launch, 'camera:=camera_3']
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        self.marker_detect_camera_3 = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        

        self.running_processes = {}
    def on_shutdown(self):
        for file in self.running_processes.values():
            file.shutdown()
    def camera_driver(self):
        """Camera Node code for multiple cameras"""
        print("="*36,"      Starting Camera Node","="*36, sep="\n" )
        # Prompt the user to enter a camera number
        while True:
            camera_num = input("Enter a camera number (1-3) or 'q' to quit: ")
            if camera_num == '1':
                # run camera 1
                self.cam1_driver.start()
                self.running_processes.update({"cam1_driver": self.cam1_driver})
                # if user press s, stop the camera node and break the loop 
                rospy.sleep(2)
                option = input("\nPress 's' to stop the camera node: ")
                if option == 's':
                    self.cam1_driver.shutdown()
                    self.running_processes.pop("cam1_driver")
                    break
            elif camera_num == '2':
                # run camera 2
                self.cam2_driver.start()
                self.running_processes.update({"cam2_driver": self.cam2_driver})
                rospy.sleep(2)
                option = input("\nPress 's' to stop the camera node: ")
                if option == 's':
                    self.cam2_driver.shutdown()
                    self.running_processes.pop("cam2_driver")
                    break
            elif camera_num == '3':
                # run camera 3
                self.cam3_driver.start()
                self.running_processes.update({"cam3_driver": self.cam3_driver})
                rospy.sleep(2)
                option = input("\nPress 's' to stop the camera node: ")
                if option == 's':
                    self.cam3_driver.shutdown()
                    self.running_processes.pop("cam3_driver")
                    break
                    
            elif camera_num == 'q':
                # quit
                break
            else:
                # Invalid camera number
                print("Invalid camera number.")
                continue
            return

    def camera_calibration(self):
        """Launches a camera calibration process for a specified camera."""
        print("="*36,"      Starting Camera Calibration","="*36, sep="\n" )
        # Launch parent processes for each camera calibration
        launch_files = [f"{self.launch_path}calib{num}.launch" for num in range(1, 4)]
        calibrations = {f"cam{num}_calib": roslaunch.parent.ROSLaunchParent(self.uuid, [launch_files[num-1]]) for num in range(1, 4)}

        # Prompt the user to enter a camera number
        while True:
            camera_num = input("Enter a camera number (1-3) to calibrate, \nor enter 'q' to quit: ")
            if camera_num == 'q':
                break
            elif camera_num not in ['1', '2', '3']:
                print("Invalid camera number.")
                continue

            # Start calibration process and wait for it to finish
            calibration = calibrations[f"cam{camera_num}_calib"]
            calibration.start()
            self.running_processes.update({f"cam{camera_num}_calib": calibration})
            while calibration.pm.is_alive():
                time.sleep(1)
            calibration.shutdown()
            del self.running_processes[f"cam{camera_num}_calib"]
            break
    def record_bag_cam(self):
        """Launches a camera bag recording process for a specified camera."""
        print("="*47,"  Recording Camera Stream for Post Processing","="*47, sep="\n" )
        while True:
            camera_num = input("Enter a camera number (1-3) for saving data for post processing,\nor 'q' for quit: ")
            time_dur_bag = int(input("\nEnter the duration in seconds for recording data: "))
            if camera_num == 'q':
                break
            
            if camera_num not in ['1', '2', '3']:
                print("Invalid camera number.")
                continue
            
            # Start the driver for the selected camera
            driver_name = f"cam{camera_num}_driver"
            driver = getattr(self, driver_name)
            driver.start()
            self.running_processes[driver_name] = driver
            time.sleep(1)

            # Start recording the bag file for the selected camera
            bagfile_name = f"cam{camera_num}_bagfile"
            cli_args = [self.record_bag_launch, f'cam:=camera_{camera_num}', f'dur:={time_dur_bag}']
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
            bagfile = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
            bagfile.start()
            self.running_processes[bagfile_name] = bagfile

            # Wait for the bag file recording to finish
            while bagfile.pm.is_alive():
                time.sleep(1)

            # Remove the camera driver and bagfile from the running_processes dictionary
            driver.shutdown()
            bagfile.shutdown()
            del self.running_processes[driver_name]
            del self.running_processes[bagfile_name]
            time.sleep(1)
            print('.'*57)
            print(f"Data recorded successfully for {time_dur_bag} seconds from Camera {camera_num}.")
            print('.'*57, '\n')
            break
    def post_process_bag_file(self):
        """Launches a camera bag recording process for a specified camera."""
        print("="*46," Post Processing for Displacement Calculation","="*46, sep="\n" )

        while True:
            # camera_num = input("Enter a camera number (1-3) for bag file saving, or 'q' for quit: ")
            camera_num = 1
            print(f"Enter a camera number (1-3) for bag file saving, or 'q' for quit: {camera_num}")
            if camera_num == 'q':
                break
            dur = int(input("Enter the duration of the bag file in seconds: "))
            cam_dict_1 = {'cam'         : ['camera_1', 'camera_2', 'camera_3'],
                    'device_id'   : ['0', '1', '2'],
                    'calib_file'  : ['cam1', 'cam2', 'cam3']}
            for i in range(len(cam_dict_1['cam'])):
                cli_args = [
                    self.read_bag_launch,
                    f"cam:={cam_dict_1['cam'][i]}",
                    f'dur:={dur}'
                ]
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
                node_name = f"read_bagfile_{cam_dict_1['cam'][i]}"
                setattr(self, node_name, roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file))
            # --------------------------------------------------------
            # STEP:01  loading the bag file for camera 1
            # --------------------------------------------------------
            if camera_num == '1':
                self.read_bagfile_camera_1.start()
                self.running_processes.update({"read_bagfile_camera_1": self.read_bagfile_camera_1})
                time.sleep(1)
                print(f'### Read successfully bag file of camera {camera_num}')
                # --------------------------------------------------------
                #  STEP:02 Detecting the fiducial markers in the bag file
                # --------------------------------------------------------
                self.marker_detect_camera_1.start()
                self.running_processes.update({"marker_detect_camera_1": self.marker_detect_camera_1})
                time.sleep(1)
                print(f'### Marker detected successfully from bag file of camera {camera_num}')
                # --------------------------------------------------------------
                #  STEP:03 Recording the fiducial markers topic in the bag file
                # --------------------------------------------------------------
                topic_name = f'/cam{camera_num}_marker/fiducial_transforms'
                now = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                # construct the file paths with the current date and time
                detect_bagfile_path = os.path.join(self.bagfile_path, f'detect_camera_{camera_num}_{dur}s_{now}.bag')
                print(f'### saving detected bag file to {detect_bagfile_path}')

                self.command = f'rosbag record -O {detect_bagfile_path} {topic_name}'
                self.process = subprocess.Popen(self.command.split(), stdout=subprocess.PIPE)

                time.sleep(dur)
                self.process.terminate()
                self.running_processes["read_bagfile_camera_1"].shutdown()
                self.running_processes["marker_detect_camera_1"].shutdown()
                print(f'### Bag file saved successfully for the detected marker from the file of camera {camera_num}')


                csv_file_path = f'/home/agcam/ros_ws/src/gige_cam_driver/data/detect_camera_{camera_num}_{dur}s_{now}.csv'

                # Use the subprocess module to execute the rostopic command and redirect the output to the CSV file
                with open(csv_file_path, 'w') as csv_file:
                    subprocess.run(['rostopic', 'echo', '-b', detect_bagfile_path, '-p', topic_name], stdout=csv_file)

                # wait for the nodes to shut down
                subprocess.call(['rosnode', 'cleanup'])
                print('\nCSV file saved successfully to:\n', csv_file_path)

                # -----------------------------------------------------------
                #  STEP:04 Saving the plot of displacements in the bag file
                # -----------------------------------------------------------

                data = pd.read_csv(csv_file_path)

                # Extract the columns of interest
                image_seq = data["field.image_seq"]
                x = data["field.transforms0.transform.translation.x"]
                y = data["field.transforms0.transform.translation.y"]
                z = data["field.transforms0.transform.translation.z"]

                # Create a new figure and set its size
                fig, axs = plt.subplots(nrows=3, figsize=(10, 8))
                # Set the title of the figure
                fig.suptitle(f"3D displacement using ARUCO maker detection - Camera {camera_num} ({dur}s) - {now}")

                # Plot the data on each subplot
                axs[0].plot(image_seq, x, color="red")
                axs[0].set_ylabel("x-axis")

                axs[1].plot(image_seq, y, color="green")
                axs[1].set_ylabel("y-axis")

                axs[2].plot(image_seq, z, color="blue")
                axs[2].set_ylabel("z-axis")

                # Add a legend and axis labels to the last subplot
                axs[2].set_xlabel("Image Sequence")
                axs[2].legend(loc="best")

                # Save the plot with a given filename
                fig.savefig(f"/home/agcam/ros_ws/src/gige_cam_driver/data/3D_disp_camera_{camera_num}_{dur}s_{now}.png")
                print('\nClose the figure to continue...')
                plt.show()
                print('\nDisplacemetn plot saved successfully to: \n', f"/home/agcam/ros_ws/src/gige_cam_driver/data/3D_disp_camera_{dur}s_{now}.png", '\n')
                print('.'*54, '\nPost processing completed successfully, now exiting...\n', '.'*54, '\n')
                # print('Post processing completed successfully, now exiting...')

                time.sleep(1)
            else:
                print("Invalid input")

            break

    
    
                
if __name__ == '__main__':

    print('\n','='*57, '\n  3D Displacement calculation using ARUCO maker detection\n', '='*57)


    while True:
        # print("\n")

        print("  1. Start Camera Calibration")
        print("  2. Save Camera data for post processing")
        print("  3. Displacement calculation using post processing")
        # print("4. Real-time displacement calculation")
        # print('4. Show live camera feed')
        print("  4. Exit")
        choice = int(input(" Please select an option [1-4]: "))
        if choice == 1:
            print('\n--------')
            print(f"Option {choice} selected, Starting camera calibration...")
            print('--------\n')
            launch_handle = LaunchHandle()
            launch_handle.camera_calibration()
            print('-----------------------------')
            print('Camera calibration completed ')
            print('-----------------------------\n')
            break
        elif choice == 2:
            print('\n--------')
            print(f"Option {choice} selected, Saving camera data for post processing...")
            print('--------\n')
            launch_handle = LaunchHandle()
            launch_handle.record_bag_cam()
            break
        elif choice == 3:
            print('\n--------')
            print(f"Option {choice} selected, Saving camera data for post processing...")
            print('--------\n')
            launch_handle = LaunchHandle()
            launch_handle.post_process_bag_file()
            break
        elif choice == 4:
            print("Exiting...")
            exit()
        else:
            print("Invalid input")
