#/usr/bin/env python3
import roslaunch
import rospkg
import rospy
import time
import subprocess
import pandas as pd
import matplotlib.pyplot as plt
import os


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

        self.bag_cam_launch   = f"{self.launch_path}bag.launch"
        self.read_bag_launch  = f"{self.launch_path}readbag.launch"
        self.cam_launch    = f"{self.launch_path}cam.launch"
        self.detect_launch = f"{self.detect_launch_path}detect.launch"
        
           
        self.cam1_calib_file_path = f"{self.launch_path}calib1.launch"
        self.cam2_calib_file_path = f"{self.launch_path}calib2.launch"
        self.cam3_calib_file_path = f"{self.launch_path}calib3.launch"
        
        
        # ********************************************************************************
        # Camera Calibration code for multiple cameras
        # ********************************************************************************
        self.cam1_calib = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam1_calib_file_path])
        self.cam2_calib = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam2_calib_file_path])
        self.cam3_calib = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam3_calib_file_path])



        # ********************************************************************************
        # Saving bag files for multiple cameras
        # ********************************************************************************

        # running ros_bag launch file for camera_1
        cli_args = [self.bag_cam_launch, 'cam:=camera_1', 'dur:=30']
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        self.cam1_bagfile = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        # running ros_bag launch file for camera_2
        cli_args = [self.bag_cam_launch, 'cam:=camera_2','dur:=30']
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        self.cam2_bagfile = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        # running ros_bag launch file for camera_3
        cli_args = [self.bag_cam_launch, 'cam:=camera_3', 'dur:=30']
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        self.cam3_bagfile = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        # ********************************************************************************

        
  
        
        # ********************************************************************************
        # Reading Bag file code for multiple cameras
        # ********************************************************************************
        
        # # define a list of camera names
        camera_names = ['camera_1', 'camera_2', 'camera_3']

        # # loop through the camera names and create a ROS launch parent for each one
        # cam_dict_1 = {'cam'         : ['camera_1', 'camera_2', 'camera_3'],
        #               'device_id'   : ['0', '1', '2'],
        #               'calib_file'  : ['cam1', 'cam2', 'cam3'],
        #               'dur'    : ['20', '20', '20']}
        # for i in range(len(cam_dict_1['cam'])):
        #     cli_args = [
        #         self.read_bag_launch,
        #         f"cam:={cam_dict_1['cam'][i]}",
        #         f"dur:={cam_dict_1['dur'][i]}"
        #     ]
        #     roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        #     node_name = f"read_bagfile_cam{i+1}"
        #     setattr(self, node_name, roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file))

        
        for name in camera_names:
            cli_args = [self.read_bag_launch, f'cam:={name}', 'dur:=30']
            
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
            setattr(self, f'read_bagfile_{name}', roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file))
            
        # # reading bag file for camera_1
        # cli_args = [self.read_bag_launch, 'cam:=camera_1']
        # roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        # self.read_bagfile_camera_1 = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        
        # # readding bag file for camera_2
        # cli_args = [self.read_bag_launch, 'cam:=camera_2']
        # roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        # self.read_bagfile_camera_2 = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        
        # # reading bag file for camera_3
        # cli_args = [self.read_bag_launch, 'cam:=camera_3']
        # roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        # self.read_bagfile_camera_3 = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        # ********************************************************************************
        
        
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
        

        self.running_processes = {}
    def on_shutdown(self):
        for file in self.running_processes.values():
            file.shutdown()
    def camera_driver(self):    
        print("=======================================")
        print("         Starting Camera Node")
        print("=======================================")

        # Prompt the user to enter a camera number
        while True:
            camera_num = input("Enter a camera number (1-3) or 'q' to quit: ")
            if camera_num == '1':
                # run camera 1
                self.cam1_driver.start()
                self.running_processes.update({"cam1_driver": self.cam1_driver})
            elif camera_num == '2':
                # run camera 2
                self.cam2_driver.start()
                self.running_processes.update({"cam2_driver": self.cam2_driver})
            elif camera_num == '3':
                # run camera 3
                self.cam3_driver.start()
                self.running_processes.update({"cam3_driver": self.cam3_driver})
            elif camera_num == 'q':
                # quit
                break
                
            else:
                # Invalid camera number
                print("Invalid camera number.")
                continue
        # kill all nodes
        # subprocess.call(['rosnode', 'kill', '-a'])

        # wait for the nodes to shut down
        # subprocess.call(['rosnode', 'cleanup']) 
        rospy.spin()
    def camera_calibration(self):    
        print("=======================================")
        print("      Starting Camera Calibration")
        print("=======================================")
        # Prompt the user to enter a camera number
        while True:
            camera_num = input("Enter a camera number (1-3) to calibrate, or 'q' to quit: ")
            if camera_num == '1':
                self.cam1_calib.start()
                self.running_processes.update({"cam1_calib": self.cam1_calib})
            elif camera_num == '2':
                self.cam2_calib.start()
                self.running_processes.update({"cam2_calib": self.cam2_calib})
            elif camera_num == '3':
                self.cam3_calib.start()
                self.running_processes.update({"cam3_calib": self.cam3_calib})
            elif camera_num == 'q':
                break
            else:
                print("Invalid camera number.")
                continue
            rospy.spin()
    def bag_cam_record(self): 
        print("\n=====================================================")
        print("      Recording Camera bag file for post processing  ")
        print("=====================================================\n")
        # Prompt the user to enter a camera number
        while True:
            camera_num = input("Enter a camera number (1-3) for bag file saving, or 'q' for quit: ")
            if camera_num == '1':
                self.cam1_driver.start()
                self.running_processes.update({"cam1_driver": self.cam1_driver})
                time.sleep(1)
                self.cam1_bagfile.start()
                self.running_processes.update({"cam1_bagfile": self.cam1_bagfile})
                rospy.spin()
            elif camera_num == '2':
                self.cam2_driver.start()
                self.running_processes.update({"cam2_driver": self.cam2_driver})
                time.sleep(1)
                self.cam2_bagfile.start()
                self.running_processes.update({"cam2_bagfile": self.cam2_bagfile})
                rospy.spin()
            elif camera_num == '3':
                self.cam3_driver.start()
                self.running_processes.update({"cam3_driver": self.cam3_driver})
                time.sleep(1)
                self.cam3_bagfile.start()
                self.running_processes.update({"cam3_bagfile": self.cam3_bagfile})
            elif camera_num == 'q':
                break
            else:
                print("Invalid camera number.")
                continue
            rospy.spin()
    def bag_file_read(self):
        self.read_bagfile_camera_1.start()
        self.running_processes.update({"read_bagfile_camera_1": self.read_bagfile_camera_1})
        # time.sleep(1)
        rospy.spin()
        
    def post_process_bag_file(self): 
        print("\n===================================")
        print("      Post Processing the bag file   ") 
        print("=====================================\n")
        # Prompt the user to enter a camera number
        while True:
            camera_num = input("Enter a camera number (1-3) for bag file saving, or 'q' for quit: ")
            if camera_num == '1':
                # # # loading the bag file for camera 1
                self.read_bagfile_camera_1.start()
                self.running_processes.update({"read_bagfile_camera_1": self.read_bagfile_camera_1})
                # rospy.spin()
                time.sleep(1)
                self.marker_detect_camera_1.start()
                self.running_processes.update({"marker_detect_camera_1": self.marker_detect_camera_1})
                topic_name = '/cam1_marker/fiducial_transforms'
                self.bagfile_path = os.path.join(self.bagfile_path, 'detect_camera_1.bag')
        
        
        self.command = f'rosbag record -O {self.bagfile_path} {topic_name}'
        self.process = subprocess.Popen(self.command.split(), stdout=subprocess.PIPE)

        time.sleep(10)
        self.process.terminate()
        self.running_processes["read_bagfile_camera_1"].shutdown()
        self.running_processes["marker_detect_camera_1"].shutdown()
        
        bag_file_path = '/home/agcam/ros_ws/src/gige_cam_driver/bagfiles/detect_camera_1.bag'
        topic_name = '/cam1_marker/fiducial_transforms'
        csv_file_path = '/home/agcam/ros_ws/src/gige_cam_driver/data/detect_camera_1.csv'

        # Use the subprocess module to execute the rostopic command and redirect the output to the CSV file
        with open(csv_file_path, 'w') as csv_file:
            subprocess.run(['rostopic', 'echo', '-b', bag_file_path, '-p', topic_name], stdout=csv_file)
            
    
        # kill all nodes
        subprocess.call(['rosnode', 'kill', '-a'])

        # wait for the nodes to shut down
        subprocess.call(['rosnode', 'cleanup'])
        rospy.spin()
        
        # # # loading the bag file for camera 2
        # self.read_bagfile_camera_2.start()
        # self.running_processes.update({"read_bagfile_camera_2": self.read_bagfile_camera_2})
        # rospy.spin()


        # # loading the bag file for camera 3
        # self.read_bagfile_camera_3.start()
        # self.running_processes.update({"read_bagfile_camera_3": self.read_bagfile_camera_3})
        # rospy.spin()
    def bag_to_csv(self):
        # print("test")
        # print(self.launch_path)
        # self.launch_path_new = self.launch_path.replace("launch/", "bagfiles/")
        # print(self.launch_path_new)
        # # path = "/home/agcam/ros_ws/src/gige_cam_driver/launch/"
        # # new_path = path.replace("launch/", "")
        bag_file_path = '/home/agcam/ros_ws/src/gige_cam_driver/bagfiles/detect_camera_1.bag'
        topic_name = '/cam1_marker/fiducial_transforms'
        csv_file_path = '/home/agcam/ros_ws/src/gige_cam_driver/data/detect_camera_1.csv'

        # Use the subprocess module to execute the rostopic command and redirect the output to the CSV file
        with open(csv_file_path, 'w') as csv_file:
            subprocess.run(['rostopic', 'echo', '-b', bag_file_path, '-p', topic_name], stdout=csv_file)
    def plot(self):


        # Load data from CSV file
        data = pd.read_csv("/home/agcam/ros_ws/src/gige_cam_driver/data/detect_camera_1.csv")

        # Extract the columns of interest
        image_seq = data["field.image_seq"]
        x = data["field.transforms0.transform.translation.x"]
        y = data["field.transforms0.transform.translation.y"]
        z = data["field.transforms0.transform.translation.z"]

        # Create a new figure and set its size
        fig, axs = plt.subplots(nrows=3, figsize=(10, 8))

        # Plot the data on each subplot
        axs[0].plot(image_seq, x, color="red")
        axs[0].set_ylabel("X")

        axs[1].plot(image_seq, y, color="green")
        axs[1].set_ylabel("Y")

        axs[2].plot(image_seq, z, color="blue")
        axs[2].set_ylabel("Z")

        # Add a legend and axis labels to the last subplot
        axs[2].set_xlabel("Image Sequence")
        axs[2].legend(loc="best")


        # # Add annotations to the plot
        # ax.annotate("X", xy=(image_seq[0], x[0]), xytext=(image_seq[0] + 5, x[0] + 5),
        #             arrowprops=dict(facecolor="red", shrink=0.05))
        # ax.annotate("Y", xy=(image_seq[0], y[0]), xytext=(image_seq[0] + 5, y[0] - 5),
        #             arrowprops=dict(facecolor="green", shrink=0.05))
        # ax.annotate("Z", xy=(image_seq[0], z[0]), xytext=(image_seq[0] + 5, z[0] + 5),
        #             arrowprops=dict(facecolor="blue", shrink=0.05))

        # Show the plot
        plt.show()




        


        
        # # self.running_processes["cam1_calib"].shutdown()
    
        
if __name__ == '__main__':
    launch_handle = LaunchHandle()
    # launch_handle.run()
    # launch_handle.bag_to_csv()
    # launch_handle.bag_file_read()
    # launch_handle.camera_driver()
    # launch_handle.camera_calibration()    # this one is working
    launch_handle.bag_cam_record()        # this one is working
    # launch_handle.plot()                  # this one is working
    
    
    