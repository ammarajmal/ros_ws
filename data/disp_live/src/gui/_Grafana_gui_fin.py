#!/usr/bin/env python3
""" backend definitions for the gui"""
# import subprocess
import tkinter as tk
import customtkinter

import os
import time
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from fiducial_msgs.msg import FiducialTransformArray
from multiprocessing import Process, Event  # Import multiprocessing

import rospy
import rospkg
import roslaunch
themes = {'blue': ("#3B8ED0", "#1F6AA5"),
          'green': ("#2CC985", "#2FA572"),
          'dark-blue': ("#3a7ebf", "#1f538d")
          }
# select value of COLOR_SELECT from (0: blue, 1: green, 2: dark-blue)
COLOR_SELECT = list(themes.keys())[0]

# Modes: "System" (standard), "Dark", "Light"
customtkinter.set_appearance_mode("System")
# Themes: "blue" (standard), "green", "dark-blue"
customtkinter.set_default_color_theme(COLOR_SELECT)
class ClientGUI(customtkinter.CTk):
    """ class for client gui code"""   
    def __init__(self)-> None:
        """initialization function for the client gui
        """
        super().__init__()
        rospy.init_node('data_recording', anonymous=True)
        self.is_recording = False  # Flag to control data processing
        # InfluxDB 2.0 setup
        token = os.environ.get("INFLUXDB_TOKEN")
        org = "Chung-Ang University"
        url = "http://localhost:8086"
        self.write_client = InfluxDBClient(url=url, token=token, org=org)
        self.write_api = self.write_client.write_api(write_options=SYNCHRONOUS)

        # Initialize variables to store the first data point for each nuc
        self.first_translation_x_nuc1 = None
        self.first_translation_y_nuc1 = None
        self.first_translation_z_nuc1 = None

        self.first_translation_x_nuc2 = None
        self.first_translation_y_nuc2 = None
        self.first_translation_z_nuc2 = None

        self.first_translation_x_nuc3 = None
        self.first_translation_y_nuc3 = None
        self.first_translation_z_nuc3 = None

        self.title("Main Dashboard")
        self.geometry("1000x600")
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.destroy_routine)

        # Create a BooleanVar to use as the variable for the checkbox
        self._create_widgets()

        

    def destroy_routine(self) -> None:
        """_summary_
        """
        self.destroy()
        self.quit()
    def _create_widgets(self) -> None:
        """_summary_
        """
        self._create_left_frame()
        self._create_right_frame()
    def _create_left_frame(self) -> None:
        """_summary_
        """
        self.left_frame = tk.Frame(self, bg=themes[COLOR_SELECT][1])
        self.left_frame.place(relx=0, rely=0, relwidth=0.25, relheight=1)
        self._create_left_top_frame()
        self._create_left_middle_frame()
        self._create_left_bottom_frame()
    def _create_left_top_frame(self) -> None:
        """_summary_"""
        self.left_top_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_top_frame.place(relx=0.1, rely=0.03, relwidth=.8, relheight=0.30)
        self._create_left_top_frame_content()
    def _create_left_top_frame_content(self)-> None:
        """_summary_
        """
        self.left_top_frame_label = customtkinter.CTkLabel(
            self.left_top_frame, text="Start Data Recording")
        self.left_top_frame_label.place(relx=0.5, rely=0.15, anchor="center")

        self.left_top_frame_start_rec_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start Record",
            fg_color=themes[COLOR_SELECT][1],
            command=self._start_record_button_event)
        self.left_top_frame_start_rec_button.place(relx=0.5, rely=0.45, anchor="center")
        
        
        self.left_top_frame_stop_rec_button = customtkinter.CTkButton(
            self.left_top_frame, text="Stop Record",
            fg_color=themes[COLOR_SELECT][1],
            command=self._stop_record_button_event)
        self.left_top_frame_stop_rec_button.place(relx=0.5, rely=0.7, anchor="center")






    def _start_nuc_cam_button_event(self, camera_number, show_camera, calibrate_camera) -> None:
        """This function is used to start the camera node"""
        pass
    
    def _create_left_middle_frame(self) -> None:
        """_summary_"""
        self.left_middle_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_middle_frame.place(relx=0.1, rely=0.36, relwidth=0.8, relheight=0.15)
        
        self._create_left_middle_frame_content()
    def _create_left_middle_frame_content(self) -> None:
        """_summary_
        """
        self.left_middle_frame_label = customtkinter.CTkLabel(
            self.left_middle_frame, text="START REMOTE CAMERAS")
        self.left_middle_frame_label.place(relx=0.5, rely=0.25, anchor="center")

        self.left_middle_frame_cams_button = customtkinter.CTkButton(
            self.left_middle_frame, text="Start All Cameras",
            fg_color=themes[COLOR_SELECT][1],
            command=lambda: self._start_nuc_cam_button_event(1, True, False))
        self.left_middle_frame_cams_button.place(relx=0.5, rely=0.6, anchor="center")

    def _create_left_bottom_frame(self) -> None:
        """_summary_"""
        self.left_bottom_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_bottom_frame.place(relx=0.1, rely=0.54, relwidth=0.8, relheight=0.35)
        self._create_left_bottom_frame_content()
    def _create_left_bottom_frame_content(self) -> None:
        """_summary_
        """
        self.left_bottom_frame_label = customtkinter.CTkLabel(
            self.left_bottom_frame, text="DETECTION PARAMETERS")
        self.left_bottom_frame_label.grid(row=0, column=0, padx=10, pady=(10,0), sticky="nsew")

        self.left_bottom_frame_sq_size_label = customtkinter.CTkLabel(
            self.left_bottom_frame, text="Marker Size (m): ")
        self.left_bottom_frame_sq_size_label.grid(row=1, column=0, padx=35, pady=(0,0), sticky="nsew")

        self.left_bottom_frame_sq_size_entry = customtkinter.CTkEntry(
            self.left_bottom_frame)
        self.left_bottom_frame_sq_size_entry.grid(row=2, column=0, padx=35, pady=(0,0), sticky="nsew")

        self.left_bottom_frame_chessboard_label = customtkinter.CTkLabel(
            self.left_bottom_frame, text="Dictionary:")
        self.left_bottom_frame_chessboard_label.grid(row=3, column=0, padx=35, pady=(0,0), sticky="nsew")

        self.left_bottom_frame_chessboard_entry = customtkinter.CTkEntry(
            self.left_bottom_frame)
        self.left_bottom_frame_chessboard_entry.grid(row=4, column=0, padx=35, pady=(0,0), sticky="nsew")
        
        self.left_button_frame_calib_update_button = customtkinter.CTkButton(
            self.left_bottom_frame, text="Update", fg_color="green")
        self.left_button_frame_calib_update_button.grid(row=5, column=0, padx=35, pady=(10,0), sticky="nsew")
        

        
        
    
        
        
    def _create_right_frame(self) -> None:
        """_summary_
        """
        self.right_frame = tk.Frame(self, bg=themes[COLOR_SELECT][1])
        self.right_frame.place(relx=0.25, rely=0, relwidth=0.75, relheight=1)
        self._create_right_top_frame()
        self._create_right_middle_frame()
        self._create_right_bottom_frame()

    def _create_right_top_frame(self) -> None:
        """_summary_
        """
        self.right_top_frame = tk.Frame(self.right_frame, bg=themes[COLOR_SELECT][0])
        self.right_top_frame.place(relx=0.01, rely=.03, relwidth=.94, relheight=0.08)
        self._create_right_top_frame_content()
    def _create_right_top_frame_content(self) -> None:
        """_summary_
        """
        self.right_top_frame_system_label = customtkinter.CTkLabel(
            self.right_top_frame, text=" System:  ")
        self.right_top_frame_system_label.place(relx=0.05, rely=0.5, anchor="center")
        self.right_top_frame_label = customtkinter.CTkLabel(
            self.right_top_frame, text=" Main PC ", text_color="yellow", bg_color=themes[COLOR_SELECT][1])
        self.right_top_frame_label.place(relx=0.12, rely=0.5, anchor="center")
        self.right_top_frame_ros_status_label = customtkinter.CTkLabel(
            self.right_top_frame, text="ROS System Status: ")
        self.right_top_frame_ros_status_label.place(relx=0.3, rely=0.5, anchor="center")
        self.right_top_frame_ros_status_result_label = customtkinter.CTkLabel(
            self.right_top_frame, text="Running", text_color="white")
        self.right_top_frame_ros_status_result_label.place(relx=0.42, rely=0.5, anchor="center")
        self.right_top_frame_camera_label = customtkinter.CTkLabel(
            self.right_top_frame, text="Active Camera: ")
        self.right_top_frame_camera_label.place(relx=0.6, rely=0.5, anchor="center")
        self.right_top_frame_camera_result_label = customtkinter.CTkLabel(
            self.right_top_frame, text="-", text_color="white")
        self.right_top_frame_camera_result_label.place(relx=0.72, rely=0.5, anchor="center")
        
    def _create_right_middle_frame(self) -> None:
        """ frame for aruco detection details   """
        self.right_middle_frame = tk.Frame(self.right_frame, bg=themes[COLOR_SELECT][0])
        self.right_middle_frame.place(relx=0.01, rely=.14, relwidth=.94, relheight=0.08)
        self._create_right_middle_frame_content()
    def _create_right_middle_frame_content(self) -> None:
        self.right_middle_frame_button_detection = customtkinter.CTkButton(self.right_middle_frame,
                                                                           text='Start Detection Process',
                                                                           fg_color=themes[COLOR_SELECT][1])
        self.right_middle_frame_button_detection.place(relx=0.1, rely=0.3)
         
    def _create_right_bottom_frame(self) -> None:
        """_summary_
        """
        self.right_bottom_frame = tk.Frame(self.right_frame, bg=themes[COLOR_SELECT][0])
        self.right_bottom_frame.place(relx=.01, rely=0.26, relwidth=.94, relheight=0.62)
    def callback_nuc1(self, data):
        self.process_data(data, 'nuc1')

    def callback_nuc2(self, data):
        self.process_data(data, 'nuc2')

    def callback_nuc3(self, data):
        self.process_data(data, 'nuc3')

    def process_data(self, data, nuc_name):
        # self.first_translation_x_nuc1, self.first_translation_y_nuc1, self.first_translation_z_nuc1
        # self.first_translation_x_nuc2, self.first_translation_y_nuc2, self.first_translation_z_nuc2
        # self.first_translation_x_nuc3, self.first_translation_y_nuc3, self.first_translation_z_nuc3

        for transform in data.transforms:
            translation = transform.transform.translation
            rotation = transform.transform.rotation

            if nuc_name == 'nuc1':
                # Process data for nuc1
                if self.first_translation_x_nuc1 is None:
                    self.first_translation_x_nuc1 = translation.x
                    self.first_translation_y_nuc1 = translation.y
                    self.first_translation_z_nuc1 = translation.z

                # Subtract the first data point for nuc1
                normalized_translation_x = translation.x - self.first_translation_x_nuc1
                normalized_translation_y = translation.y - self.first_translation_y_nuc1
                normalized_translation_z = translation.z - self.first_translation_z_nuc1

            elif nuc_name == 'nuc2':
                # Process data for nuc2
                if self.first_translation_x_nuc2 is None:
                    self.first_translation_x_nuc2 = translation.x
                    self.first_translation_y_nuc2 = translation.y
                    self.first_translation_z_nuc2 = translation.z

                # Subtract the first data point for nuc2
                normalized_translation_x = translation.x - self.first_translation_x_nuc2
                normalized_translation_y = translation.y - self.first_translation_y_nuc2
                normalized_translation_z = translation.z - self.first_translation_z_nuc2

            elif nuc_name == 'nuc3':
                # Process data for nuc3
                if self.first_translation_x_nuc3 is None:
                    self.first_translation_x_nuc3 = translation.x
                    self.first_translation_y_nuc3 = translation.y
                    self.first_translation_z_nuc3 = translation.z

                # Subtract the first data point for nuc3
                normalized_translation_x = translation.x - self.first_translation_x_nuc3
                normalized_translation_y = translation.y - self.first_translation_y_nuc3
                normalized_translation_z = translation.z - self.first_translation_z_nuc3

            # Save normalized data to InfluxDB
            self.save_to_influxdb(nuc_name, normalized_translation_x, normalized_translation_y, normalized_translation_z, rotation)

    def save_to_influxdb(self, nuc_name, translation_x, translation_y, translation_z, rotation):
        point = Point("fiducial_transforms") \
            .tag("nuc", nuc_name) \
            .field("translation_x", translation_x) \
            .field("translation_y", translation_y) \
            .field("translation_z", translation_z) \
            .field("rotation_x", rotation.x) \
            .field("rotation_y", rotation.y) \
            .field("rotation_z", rotation.z) \
            .field("rotation_w", rotation.w) \
            .time(time.time_ns(), WritePrecision.NS)
        self.write_api.write(bucket="SITL", record=point)
if __name__ == "__main__":
    root = ClientGUI()
    root.mainloop()
    