#!/usr/bin/env python3
""" backend definitions for the gui"""
import os
import influxdb_client
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
from fiducial_msgs.msg import FiducialTransformArray
import subprocess
import tkinter as tk
import customtkinter
import rospy
import rospkg
import roslaunch
import time
from _backend_ import *
from _backend_ import _check_ros_status_function
import webbrowser



# Modes: "System" (standard), "Dark", "Light"
customtkinter.set_appearance_mode("System")
# Themes: "blue" (standard), "green", "dark-blue"
customtkinter.set_default_color_theme(COLOR_SELECT)

class ClientGUI(customtkinter.CTk):
    """ class for client gui code"""
    def __init__(self) -> None:
        """initialization function for the client gui
        """
        super().__init__()
        rospy.init_node("server_gui", anonymous=False)
        
        # InfluxDB 2.0 setup
        self.token = os.environ.get("INFLUXDB_TOKEN")
        self.org = "Chung-Ang University"
        self.url = "http://localhost:8086"
        self.write_client = InfluxDBClient(url=self.url, token=self.token, org=self.org)
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
        
        self.package = 'gige_cam_driver'
        # ********************************************************************************
        # Path management for Launch files
        # ********************************************************************************

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(self.uuid)

        self.title("Displacements Dashboard")
        self.geometry("1000x600")
        self.resizable(False, False)
        self.left_width = 0.40
        self.protocol("WM_DELETE_WINDOW", self.destroy_routine)
        # Create a BooleanVar to use as the variable for the checkbox
        self.exp_name = "Experiment_1"
        self.running_processes = {}
        self.left_frame = None
        self.left_top_frame = None
        self.left_top_frame_label = None
        self.left_top_frame_button = None
        self.left_bottom_frame = None
        self.left_bottom_frame_label = None
        self.left_bottom_frame_start_calib_button = customtkinter.CTkButton(
            master=self.left_bottom_frame)
        self.left_bottom_frame_experiment_label = None
        self.left_bottom_frame_experiment_entry = None
        self.right_top_frame_system_label = None
        self.right_top_frame_label = None
        self.left_top_frame_view_cam_checkbox = None
        self.left_button_frame_marker_update_button = customtkinter.CTkButton(
            master=self.left_bottom_frame)
        self.right_frame = None
        self.right_top_frame = None
        self.right_top_frame_ros_status_result_label = None
        self.right_top_frame_camera_label = None
        self.right_top_frame_camera_result_label = None
        self.right_top_frame_ros_sys_status_button = None
        self.right_middle_frame = None
        self.left_middle_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_middle_frame_label = customtkinter.CTkLabel(self.left_middle_frame)
        self.left_button_frame_maker_update_label = customtkinter.CTkLabel(self.left_bottom_frame)
        
        self._create_widgets()

    def destroy_routine(self) -> None:
        """_summary_"""
        self.destroy()
        self.quit()

    def _create_widgets(self) -> None:
        """_summary_"""
        self._create_left_frame()
        self._create_right_frame()

    def _create_left_frame(self) -> None:
        """_summary_"""
        self.left_frame = tk.Frame(self, bg=themes[COLOR_SELECT][1])
        self.left_frame.place(relx=0, rely=0, relwidth=self.left_width, relheight=1)
        self._create_left_top_frame()
        self._create_left_bottom_frame()
    def _create_left_top_frame(self) -> None:
        """_summary_"""
        self.left_top_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_top_frame.place(relx=0.1, rely=0.04, relwidth=0.8, relheight=0.30)
        self._create_left_top_frame_content()
    def _create_left_top_frame_content(self) -> None:
        """_summary_"""
        self.left_top_frame_label = customtkinter.CTkLabel(
            self.left_top_frame, text="START DATA LOGGING")
        self.left_top_frame_label.place(relx=0.5, rely=0.15, anchor="center")

        self.left_top_frame_start_data_server_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start Data Server",
            command=self._start_data_server_event)
        self.left_top_frame_start_data_server_button.place(relx=0.5, rely=0.4, anchor="center")
        
    def _start_data_server_event(self) -> None:
        """_summary_"""
        # print("Starting Data Server")
        # rospy.init_node('data_processor', anonymous=True)
        # rospy.Subscriber("/nuc1/fiducial_transforms", FiducialTransformArray, callback_nuc1)
        # rospy.Subscriber("/nuc2/fiducial_transforms", FiducialTransformArray, callback_nuc2)
        # rospy.Subscriber("/nuc3/fiducial_transforms", FiducialTransformArray, callback_nuc3)
        # try:
        #     rospy.spin()
        # finally:
        #     write_api.close()
        #     write_client.close()
        self.open_url()

    def open_url(self):
        url = "http://localhost:3000/d/d6afadb7-6453-432c-870f-9758a492d2e7/all-nuc-displaement?orgId=1"
        webbrowser.open(url)


    def _create_left_bottom_frame(self) -> None:
        """_summary_
        """
        self.left_bottom_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_bottom_frame.place(relx=0.1, rely=0.50, relwidth=0.8, relheight=0.32)
        self._create_left_bottom_frame_content()
    def _create_left_bottom_frame_content(self) -> None:
        """_summary_
        """
        self.left_bottom_frame_label = customtkinter.CTkLabel(
        self.left_bottom_frame, text="FILE MANAGEMENT")
        self.left_bottom_frame_label.place(relx=0.5, rely=0.13, anchor="center")

        self.left_bottom_frame_experiment_label = customtkinter.CTkLabel(
            self.left_bottom_frame, text="Experiment Name:")
        self.left_bottom_frame_experiment_label.place(relx=0.1, rely=0.22)

        self.left_bottom_frame_experiment_entry = customtkinter.CTkEntry(
            master=self.left_bottom_frame,
            placeholder_text=self.exp_name,
            placeholder_text_color="gray",
            # state="disabled"
        )
        self.left_bottom_frame_experiment_entry.place(relx=0.50, rely=0.22, relwidth=0.35)

        self.left_button_frame_marker_update_button = customtkinter.CTkButton(
            self.left_bottom_frame, text="Update")
        self.left_button_frame_marker_update_button.place(relx=0.5, rely=0.65,
                                                         relwidth=0.4, anchor="center")

    def _create_right_frame(self) -> None:
        """_summary_
        """
        self.right_frame = tk.Frame(self, bg=themes[COLOR_SELECT][1])
        self.right_frame.place(relx=self.left_width, rely=0, relwidth=1-self.left_width, relheight=1)
        self._create_right_top_frame()
        self._create_right_middle_frame()
        # self._create_right_lower_frame()
    def _create_right_top_frame(self) -> None:
        """_summary_
        """
        self.right_top_frame = tk.Frame(self.right_frame, bg=themes[COLOR_SELECT][0])
        self.right_top_frame.place(relx=0.01, rely=.04, relwidth=.94, relheight=0.08)
        self._create_right_top_frame_content()
    def _create_right_top_frame_content(self) -> None:
        """_summary_
        """
        self.right_top_frame_system_label = customtkinter.CTkLabel(
            self.right_top_frame, text=" System:  ")
        self.right_top_frame_system_label.place(relx=0.15, rely=0.5, anchor="center")
        self.right_top_frame_label = customtkinter.CTkLabel(
            self.right_top_frame, text="  Main PC  ", text_color="yellow")
        self.right_top_frame_label.place(relx=0.24, rely=0.5, anchor="center")
        self.right_top_frame_ros_sys_status_button = customtkinter.CTkButton(
            self.right_top_frame, text="Camera Status:",
            fg_color=themes[COLOR_SELECT][1], border_color='gray', border_width=1,
            command= lambda: _check_ros_status_function(self.right_top_frame_ros_status_result_label))
        self.right_top_frame_ros_sys_status_button.place(relx=0.45, rely=0.5, anchor='center', relwidth=0.18)
        self.right_top_frame_ros_status_result_label = customtkinter.CTkLabel(
            self.right_top_frame, text="No Active Camera", text_color="yellow")
        self.right_top_frame_ros_status_result_label.place(relx=0.72, rely=0.5, anchor="center")

    def _create_right_middle_frame(self) -> None:
        """_summary_
        """
        self.right_middle_frame = customtkinter.CTkFrame(self.right_frame, fg_color=themes[COLOR_SELECT][0])
        self.right_middle_frame.place(relx=.01, rely=0.15, relwidth=.94, relheight=0.30)
        self._createright_middle_frame_content()
    def _create_right_lower_frame(self) -> None:
        """_summary_
        """
        self.right_lower_frame = tk.Frame(self.right_frame, bg=themes[COLOR_SELECT][0])
        self.right_lower_frame.place(relx=.01, rely=0.35, relwidth=.94, relheight=0.20)
        self._create_right_lower_frame_content()
    def _createright_middle_frame_content(self) -> None:
        """_summary_
        """
        self.right_middle_system_param_label = customtkinter.CTkLabel(
            self.right_middle_frame, text="SYSTEM PARAMETERS", text_color="white")
        self.right_middle_system_param_label.place(relx=0.5, rely=0.15, anchor="center")
            
    def _create_right_lower_frame_content(self) -> None:
        pass
    
    
    
    # def callback_nuc1(self, data):
    #     self.process_data(data, 'nuc1')

    # def callback_nuc2(self, data):
    #     self.process_data(data, 'nuc2')

    # def callback_nuc3(self, data):
    #     self.process_data(data, 'nuc3')

    # def process_data(data, nuc_name):
    #     # global first_translation_x_nuc1, first_translation_y_nuc1, first_translation_z_nuc1
    #     # global first_translation_x_nuc2, first_translation_y_nuc2, first_translation_z_nuc2
    #     # global first_translation_x_nuc3, first_translation_y_nuc3, first_translation_z_nuc3

    #     for transform in data.transforms:
    #         translation = transform.transform.translation
    #         rotation = transform.transform.rotation

    #         if nuc_name == 'nuc1':
    #             # Process data for nuc1
    #             # print(first_translation_x_nuc1)
    #             if self.first_translation_x_nuc1 is None:
    #                 self.first_translation_x_nuc1 = translation.x
    #                 self.first_translation_y_nuc1 = translation.y
    #                 self.first_translation_z_nuc1 = translation.z

    #             # Subtract the first data point for nuc1
    #             normalized_translation_x = translation.x - self.first_translation_x_nuc1
    #             normalized_translation_y = translation.y - self.first_translation_y_nuc1
    #             normalized_translation_z = translation.z - self.first_translation_z_nuc1

    #         elif nuc_name == 'nuc2':
    #             # Process data for nuc2
    #             if self.first_translation_x_nuc2 is None:
    #                 self.first_translation_x_nuc2 = translation.x
    #                 self.first_translation_y_nuc2 = translation.y
    #                 self.first_translation_z_nuc2 = translation.z

    #             # Subtract the first data point for nuc2
    #             normalized_translation_x = translation.x - self.first_translation_x_nuc2
    #             normalized_translation_y = translation.y - self.first_translation_y_nuc2
    #             normalized_translation_z = translation.z - self.first_translation_z_nuc2

    #         elif nuc_name == 'nuc3':
    #             # Process data for nuc3
    #             if self.first_translation_x_nuc3 is None:
    #                 self.first_translation_x_nuc3 = translation.x
    #                 self.first_translation_y_nuc3 = translation.y
    #                 self.first_translation_z_nuc3 = translation.z

    #             # Subtract the first data point for nuc3
    #             normalized_translation_x = translation.x - self.first_translation_x_nuc3
    #             normalized_translation_y = translation.y - self.first_translation_y_nuc3
    #             normalized_translation_z = translation.z - self.first_translation_z_nuc3

    #         # Save normalized data to InfluxDB
    #         self.save_to_influxdb(nuc_name, normalized_translation_x, normalized_translation_y, normalized_translation_z, rotation)

    # def save_to_influxdb(self, nuc_name, translation_x, translation_y, translation_z, rotation):
    #     point = Point("fiducial_transforms") \
    #         .tag("nuc", nuc_name) \
    #         .field("translation_x", translation_x) \
    #         .field("translation_y", translation_y) \
    #         .field("translation_z", translation_z) \
    #         .field("rotation_x", rotation.x) \
    #         .field("rotation_y", rotation.y) \
    #         .field("rotation_z", rotation.z) \
    #         .field("rotation_w", rotation.w) \
    #         .time(time.time_ns(), WritePrecision.NS)
    #     self.write_api.write(bucket="SITL", record=point)
    
    
    
if __name__ == "__main__":
    root = ClientGUI()
    root.mainloop()
