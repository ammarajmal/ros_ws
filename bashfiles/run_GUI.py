import tkinter as tk
from tkinter import messagebox
import datetime
import os
import subprocess
import time

import pandas as pd
import matplotlib.pyplot as plt

import roslaunch
import rospkg
import rospy


class DisplacementArucoGUI:
    """Class to store the displacement of the aruco marker"""


    def __init__(self, master):
        """Constructor of the class"""
        self.master = master
        master.title("Displacement Aruco")

        # create buttons to start and stop camera nodes
        self.start_button = tk.Button(master, text="Start Camera Nodes", command=self.start_camera_nodes)
        self.start_button.pack()

        self.stop_button = tk.Button(master, text="Stop Camera Nodes", command=self.stop_camera_nodes, state="disabled")
        self.stop_button.pack()

        # create a label to display status messages
        self.status_label = tk.Label(master, text="")
        self.status_label.pack()

        # initialize variables
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


    def start_camera_nodes(self):
        """Start camera nodes"""
        try:
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
                process = getattr(self, node_name)
                process.start()
                self.running_processes.update({node_name: process})

            self.start_button.config(state="disabled")
            self.stop_button.config(state="normal")
            self.status_label.config(text="Camera nodes started successfully.")

        except Exception as e:
            messagebox.showerror("Error", str(e))
            self.status_label.config(text=f"Error starting camera nodes: {str(e)}")

    def stop_camera_nodes(self):
        """Stop camera nodes"""
        try:
            for node_name, process in self.running_processes.items():
                process.shutdown()
           
