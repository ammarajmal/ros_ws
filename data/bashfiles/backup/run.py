#!/usr/bin/env python3

import datetime
import os
import subprocess
import time

import pandas as pd
import matplotlib.pyplot as plt

import roslaunch
import rospkg
import rospy


class DisplacementAruco:
    """Class to store the displacement of the aruco marker"""
    
    
    def __init__(self):
        """Constructor of the class"""
        rospy.init_node("displacement_aruco", anonymous=True)
        # rospy.on_shutdown(self.on_shutdown)
        
        self.running_processes = {}
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.package = 'gige_cam_driver'
        
        # path to the launch files
        self.launch_path = os.path.join(rospkg.RosPack().get_path(self.package), 'launch/')
        self.detect_path = os.path.join(rospkg.RosPack().get_path('aruco_detect'), 'launch/')
        self.bag_path = self.launch_path.replace('launch', 'bagfiles')
        
        self.record_bag_launch = os.path.join(self.launch_path, 'recordbag.launch')
        self.read_bag_launch = os.path.join(self.launch_path, 'readbag.launch')
        self.cam_launch = os.path.join(self.launch_path, 'cam.launch')
        self.detect_launch = os.path.join(self.detect_path, 'detect.launch')
        

    def camera_driver(self):
        """Camera Node code for multiple cameras"""
        print("="*36,"      Starting Camera Node","="*36, sep="\n" )
        # Prompt the user to enter a camera number
        while True:
            
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
                    continue
            elif camera_num == '2':
                # run camera 2
                self.cam2_driver.start()
                self.running_processes.update({"cam2_driver": self.cam2_driver})
                rospy.sleep(2)
                option = input("\nPress 's' to stop the camera node: ")
                if option == 's':
                    self.cam2_driver.shutdown()
                    self.running_processes.pop("cam2_driver")
                    continue
            elif camera_num == '3':
                # run camera 3
                self.cam3_driver.start()
                self.running_processes.update({"cam3_driver": self.cam3_driver})
                rospy.sleep(2)
                option = input("\nPress 's' to stop the camera node: ")
                if option == 's':
                    self.cam3_driver.shutdown()
                    self.running_processes.pop("cam3_driver")
                    continue
                    
            elif camera_num == 'q':
                # quit
                break
            else:
                # Invalid camera number
                print("Invalid camera number.")
                continue
            return

        
        
        
        
    
    def on_shutdown(self):
        """Function to be called when the node is shutdown"""
        for process in self.running_processes.values():
            process.terminate()

    def run(self):
        """Function to run the node"""
        print(self.record_bag_launch)
        print(self.read_bag_launch)
        print(self.cam_launch)
        print(self.detect_launch)
        
if __name__ == '__main__':
        disp_calc = DisplacementAruco()
        disp_calc.camera_driver()