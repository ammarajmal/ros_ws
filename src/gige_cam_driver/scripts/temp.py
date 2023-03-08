 def post_process_bag_file(self):
        """Launches a camera bag recording process for a specified camera."""
        print("="*46," Post Processing for Displacement Calculation","="*46, sep="\n" )
        # Prompt the user to enter a camera number
        
        while True:
            camera_num = input("Enter a camera number (1-3) for bag file saving, or 'q' for quit: ")
            if camera_num == 'q':
                break
            dur = int(input("Enter the duration of the bag file in seconds: "))
            # duration = [5, 10, 20, 30, 40]
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
                # --------------------------------------------------------
                #  STEP:02 Detecting the fiducial markers in the bag file
                # --------------------------------------------------------
                self.marker_detect_camera_1.start()
                self.running_processes.update({"marker_detect_camera_1": self.marker_detect_camera_1})
                
                # --------------------------------------------------------------
                #  STEP:03 Recording the fiducial markers topic in the bag file
                # --------------------------------------------------------------
                                
                topic_name = '/cam1_marker/fiducial_transforms'
                self.bagfile_path = os.path.join(self.bagfile_path, f'detect_camera_1_{str(dur)}s.bag')
        
        
                self.command = f'rosbag record -O {self.bagfile_path} {topic_name}'
                self.process = subprocess.Popen(self.command.split(), stdout=subprocess.PIPE)

                time.sleep(dur)
                self.process.terminate()
                self.running_processes["read_bagfile_camera_1"].shutdown()
                self.running_processes["marker_detect_camera_1"].shutdown()
                
                
                
                bag_file_path = f'/home/agcam/ros_ws/src/gige_cam_driver/bagfiles/detect_camera_1_{str(dur)}s.bag'
                topic_name = '/cam1_marker/fiducial_transforms'
                csv_file_path = f'/home/agcam/ros_ws/src/gige_cam_driver/data/detect_camera_1_{str(dur)}s.csv'

                # Use the subprocess module to execute the rostopic command and redirect the output to the CSV file
                with open(csv_file_path, 'w') as csv_file:
                    subprocess.run(['rostopic', 'echo', '-b', bag_file_path, '-p', topic_name], stdout=csv_file)
                    
            
                # kill all nodes
                subprocess.call(['rosnode', 'kill', '-a'])

                # wait for the nodes to shut down
                subprocess.call(['rosnode', 'cleanup'])
                print('\nCSV file saved successfully to:\n', csv_file_path)
                
                # -----------------------------------------------------------
                #  STEP:04 Saving the plot of displacements in the bag file
                # -----------------------------------------------------------
                # time.sleep(1)
                # print('Saving displacements plot in the bag file...')
                # Load data from CSV file
                # csv_file_path
                data = pd.read_csv(f"/home/agcam/ros_ws/src/gige_cam_driver/data/detect_camera_1_{dur}s.csv")

                # Extract the columns of interest
                image_seq = data["field.image_seq"]
                x = data["field.transforms0.transform.translation.x"]
                y = data["field.transforms0.transform.translation.y"]
                z = data["field.transforms0.transform.translation.z"]

                # Create a new figure and set its size
                fig, axs = plt.subplots(nrows=3, figsize=(10, 8))
                # Set the title of the figure
                fig.suptitle(f"3D displacement using ARUCO maker detection - Camera 1 ({dur}s)")
                
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
                fig.savefig(f"/home/agcam/ros_ws/src/gige_cam_driver/data/3D_displacement_ARUCO_{dur}s.png")
                print('\nClose the figure to continue...')
                plt.show()
                print('\nDisplacemetn plot saved successfully to: \n', f"/home/agcam/ros_ws/src/gige_cam_driver/data/3D_displacement_ARUCO_{dur}s.png", '\n')
                print('.'*54, '\nPost processing completed successfully, now exiting...\n', '.'*54, '\n')
                # print('Post processing completed successfully, now exiting...')
                
                time.sleep(1)
                break

                        # --------------------------------------------------------
            # STEP:01  loading the bag file for camera 2
            # --------------------------------------------------------
            elif camera_num == '2':
                self.read_bagfile_camera_2.start()
                self.running_processes.update({"read_bagfile_camera_2": self.read_bagfile_camera_2})
                time.sleep(1)
                # --------------------------------------------------------
                #  STEP:02 Detecting the fiducial markers in the bag file
                # --------------------------------------------------------
                self.marker_detect_camera_2.start()
                self.running_processes.update({"marker_detect_camera_2": self.marker_detect_camera_2})
                
                # --------------------------------------------------------------
                #  STEP:03 Recording the fiducial markers topic in the bag file
                # --------------------------------------------------------------
                                
                topic_name = '/cam2_marker/fiducial_transforms'
                self.bagfile_path = os.path.join(self.bagfile_path, f'detect_camera_2_{str(dur)}s.bag')
        
        
                self.command = f'rosbag record -O {self.bagfile_path} {topic_name}'
                self.process = subprocess.Popen(self.command.split(), stdout=subprocess.PIPE)

                time.sleep(dur)
                self.process.terminate()
                self.running_processes["read_bagfile_camera_2"].shutdown()
                self.running_processes["marker_detect_camera_2"].shutdown()
                
                
                
                bag_file_path = f'/home/agcam/ros_ws/src/gige_cam_driver/bagfiles/detect_camera_2_{str(dur)}s.bag'
                topic_name = '/cam2_marker/fiducial_transforms'
                csv_file_path = f'/home/agcam/ros_ws/src/gige_cam_driver/data/detect_camera_2_{str(dur)}s.csv'

                # Use the subprocess module to execute the rostopic command and redirect the output to the CSV file
                with open(csv_file_path, 'w') as csv_file:
                    subprocess.run(['rostopic', 'echo', '-b', bag_file_path, '-p', topic_name], stdout=csv_file)
                    
            
                # kill all nodes
                subprocess.call(['rosnode', 'kill', '-a'])

                # wait for the nodes to shut down
                subprocess.call(['rosnode', 'cleanup'])
                print('\nCSV file saved successfully to:\n', csv_file_path)
                
                # -----------------------------------------------------------
                #  STEP:04 Saving the plot of displacements in the bag file
                # -----------------------------------------------------------
                # time.sleep(1)
                # print('Saving displacements plot in the bag file...')
                # Load data from CSV file
                # csv_file_path
                data = pd.read_csv(f"/home/agcam/ros_ws/src/gige_cam_driver/data/detect_camera_2_{dur}s.csv")

                # Extract the columns of interest
                image_seq = data["field.image_seq"]
                x = data["field.transforms0.transform.translation.x"]
                y = data["field.transforms0.transform.translation.y"]
                z = data["field.transforms0.transform.translation.z"]

                # Create a new figure and set its size
                fig, axs = plt.subplots(nrows=3, figsize=(10, 8))
                # Set the title of the figure
                fig.suptitle(f"3D displacement using ARUCO maker detection - Camera 2 ({dur}s)")
                
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
                fig.savefig(f"/home/agcam/ros_ws/src/gige_cam_driver/data/3D_disp_ARUCO_camera_{camera_num}_{dur}s.png")
                print('\nClose the figure to continue...')
                plt.show()
                print('\nDisplacemetn plot saved successfully to: \n', f"/home/agcam/ros_ws/src/gige_cam_driver/data/3D_disp_ARUCO_camera_{camera_num}_{dur}s.png", '\n')
                print('.'*54, '\nPost processing completed successfully, now exiting...\n', '.'*54, '\n')
                # print('Post processing completed successfully, now exiting...')
                
                time.sleep(1)
                break
                        # --------------------------------------------------------
            # STEP:01  loading the bag file for camera 3
            # --------------------------------------------------------
            elif camera_num == '3':
                self.read_bagfile_camera_3.start()
                self.running_processes.update({"read_bagfile_camera_3": self.read_bagfile_camera_3})
                time.sleep(1)
                # --------------------------------------------------------
                #  STEP:02 Detecting the fiducial markers in the bag file
                # --------------------------------------------------------
                self.marker_detect_camera_3.start()
                self.running_processes.update({"marker_detect_camera_3": self.marker_detect_camera_3})
                
                # --------------------------------------------------------------
                #  STEP:03 Recording the fiducial markers topic in the bag file
                # --------------------------------------------------------------
                                
                topic_name = '/cam3_marker/fiducial_transforms'
                self.bagfile_path = os.path.join(self.bagfile_path, f'detect_camera_3_{str(dur)}s.bag')
        
        
                self.command = f'rosbag record -O {self.bagfile_path} {topic_name}'
                self.process = subprocess.Popen(self.command.split(), stdout=subprocess.PIPE)

                time.sleep(dur)
                self.process.terminate()
                self.running_processes["read_bagfile_camera_3"].shutdown()
                self.running_processes["marker_detect_camera_3"].shutdown()
                
                
                
                bag_file_path = f'/home/agcam/ros_ws/src/gige_cam_driver/bagfiles/detect_camera_3_{str(dur)}s.bag'
                topic_name = '/cam3_marker/fiducial_transforms'
                csv_file_path = f'/home/agcam/ros_ws/src/gige_cam_driver/data/detect_camera_3_{str(dur)}s.csv'

                # Use the subprocess module to execute the rostopic command and redirect the output to the CSV file
                with open(csv_file_path, 'w') as csv_file:
                    subprocess.run(['rostopic', 'echo', '-b', bag_file_path, '-p', topic_name], stdout=csv_file)
                    
            
                # kill all nodes
                subprocess.call(['rosnode', 'kill', '-a'])

                # wait for the nodes to shut down
                subprocess.call(['rosnode', 'cleanup'])
                print('\nCSV file saved successfully to:\n', csv_file_path)
                
                # -----------------------------------------------------------
                #  STEP:04 Saving the plot of displacements in the bag file
                # -----------------------------------------------------------
                # time.sleep(1)
                # print('Saving displacements plot in the bag file...')
                # Load data from CSV file
                # csv_file_path
                data = pd.read_csv(f"/home/agcam/ros_ws/src/gige_cam_driver/data/detect_camera_3_{dur}s.csv")

                # Extract the columns of interest
                image_seq = data["field.image_seq"]
                x = data["field.transforms0.transform.translation.x"]
                y = data["field.transforms0.transform.translation.y"]
                z = data["field.transforms0.transform.translation.z"]

                # Create a new figure and set its size
                fig, axs = plt.subplots(nrows=3, figsize=(10, 8))
                # Set the title of the figure
                fig.suptitle(f"3D displacement using ARUCO maker detection - Camera 3 ({dur}s)")
                
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
                fig.savefig(f"/home/agcam/ros_ws/src/gige_cam_driver/data/3D_disp_ARUCO_camera_{camera_num}_{dur}s.png")
                print('\nClose the figure to continue...')
                plt.show()
                print('\nDisplacemetn plot saved successfully to: \n', f"/home/agcam/ros_ws/src/gige_cam_driver/data/3D_disp_ARUCO_camera_{camera_num}_{dur}s.png", '\n')
                print('.'*54, '\nPost processing completed successfully, now exiting...\n', '.'*54, '\n')
                # print('Post processing completed successfully, now exiting...')
                
                time.sleep(1)
                break
          
            else:
                print("Invalid input")
                break
