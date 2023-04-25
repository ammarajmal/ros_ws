#!/usr/bin/env python3

# install camera sdk from: https://minsvision.com/rjxz

import datetime
import os
import subprocess
import time
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal
import numpy as np
# from PIL import Image, ImageTk
import rosbag

import tkinter as tk
from tkinter import filedialog

import roslaunch

import rospkg
import rospy
from sensor_msgs.msg import Image, CameraInfo



import customtkinter
import warnings
warnings.filterwarnings("ignore", category=FutureWarning)


# rosrun camera_calibration cameracalibrator.py --size 6x5 --square 0.025 --k-coefficients=2 --fix-principal-point i --fix-aspect-ratio image:=/camera_1/image_raw camera:=/camera_1
# subprocess.Popen(['gnome-terminal', '--', '/bin/bash', '-c',
#                  'source /opt/ros/noetic/setup.bash && roscore; exec /bin/bash'])
time.sleep(2)

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


class GUI(customtkinter.CTk):
    """_summary_

    Args:
        customtkinter (_type_): _description_
    """

    def __init__(self) -> None:
        super().__init__()

        rospy.init_node('launch_handle', anonymous=False)
        self.package = 'gige_cam_driver'

        # ********************************************************************************
        # Path management for Launch files
        # ********************************************************************************

        self.launch_path = rospkg.RosPack().get_path('gige_cam_driver') + '/launch/'
        self.csv_file_path = rospkg.RosPack().get_path('gige_cam_driver') + '/csvfiles/'
        self.detect_launch_path = rospkg.RosPack().get_path('aruco_detect') + '/launch/'
        self.bagfile_path = self.launch_path.replace("launch/", "bagfiles/")

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        self.record_bag_launch = f"{self.launch_path}recordbag.launch"
        
        self.record_multibag_launch_path = f"{self.launch_path}allbags.launch"
        
        
        self.read_bag_launch = f"{self.launch_path}readbag.launch"
        
        
        self.cam_launch = f"{self.launch_path}cam.launch"
        self.view_launch = f"{self.launch_path}viewcam.launch"
        self.calib_launch = f"{self.launch_path}calib.launch"
        self.detect_launch = f"{self.detect_launch_path}aruco_detect.launch"

        self.check_camera_1_var = tk.StringVar(self, "on")
        self.check_camera_2_var = tk.StringVar(self, "on")
        self.check_camera_3_var = tk.StringVar(self, "on")
        self.camera_selection_var = tk.StringVar(self, "Camera 1")
        self.single_camera_duration_var = tk.StringVar(self, 'Select')
        self.multi_camera_duration_var = tk.StringVar(self, 'Select')
        self.opened_bagfile_var = ''
        self.recorded_datetime_var = ''
        self.single_camera_dur = ''
        self.multi_camera_dur = ''
        self.last_recorded_bag_file_name_with_path = ''
        self.loaded_last_file_flag = False
        self.opened_file_flag = False
        self.multi_cameras_active_status = set()
        # self.last_recorded_bag_file_name_path = os.path.join(
        #             self.bagfile_path, self.last_recorded_bag_file_name)

        self.sidebar_entry_get_calib_sq_size_var = tk.StringVar()
        self.sidebar_entry_get_calib_cb_dim_var = tk.StringVar()
        self.board_size = "6x5"
        self.square_size = "0.025"
        self.maker_size = "0.1"
        self.var_marker_size = tk.StringVar(self, "0.05")
        self.var_dictionary = tk.StringVar(self, "4")
        self.running_processes = {}
        self.camera_1_active = False
        self.camera_2_active = False
        self.camera_3_active = False

        self.title("Displacement Measurement using ARUCO Marker")
        self.main_label = customtkinter.CTkLabel(
            master=self,
            text="Main Processing",
            font=customtkinter.CTkFont(size=20, weight="bold")
        )
        self.main_label.grid(row=0, column=1, padx=10,
                             pady=(10, 0), sticky="nsew")

        self.sidebar_frame = customtkinter.CTkFrame(
            master=self,
            corner_radius=0,
            fg_color='#C4C3C3'
        )
        self.sidebar_frame.grid(row=0, column=0, rowspan=11, sticky="nsew")

        self.sidebar_frame_cam_calib = customtkinter.CTkFrame(
            master=self.sidebar_frame,
            fg_color=('lightgray', 'gray')
        )
        self.sidebar_frame_cam_view = customtkinter.CTkFrame(
            master=self.sidebar_frame,
            fg_color=('lightgray', 'gray')
        )
        self.sidebar_frame_ui = customtkinter.CTkFrame(
            master=self.sidebar_frame,
            fg_color=('lightgray', 'gray')
        )
        self.sidebar_frame_cam_calib.grid(
            row=1, column=0, rowspan=5, pady=10, padx=10)
        self.sidebar_frame_cam_view.grid(
            row=6, column=0, rowspan=3, pady=10, padx=10)
        self.sidebar_frame_ui.grid(
            row=9, column=0, rowspan=2, pady=10, padx=10)
        self.sidebar_frame_cam_calib_label = customtkinter.CTkLabel(
            master=self.sidebar_frame,
            text="Camera Calibration",
            font=customtkinter.CTkFont(size=20, weight="normal"),
            text_color='black'
        )
        self.sidebar_frame_cam_calib_label.grid(
            row=0, column=0, padx=30, pady=(20, 5), sticky="nsew")
        self.sidebar_btn_cam_1_calib = customtkinter.CTkButton(
            master=self.sidebar_frame_cam_calib,
            text='Calibrate Camera 1',
            command=lambda: self.camera_calibration(1)
        )
        self.sidebar_btn_cam_2_calib = customtkinter.CTkButton(
            master=self.sidebar_frame_cam_calib,
            text='Calibrate Camera 2',
            command=lambda: self.camera_calibration(2)
        )
        self.sidebar_btn_cam_3_calib = customtkinter.CTkButton(
            self.sidebar_frame_cam_calib,
            text='Calibrate Camera 3',
            command=lambda: self.camera_calibration(3)
        )
        self.sidebar_entry_get_calib_cb_dim_label = customtkinter.CTkLabel(
            master=self.sidebar_frame_cam_calib,
            text="Board size (wxh):"
        )
        self.sidebar_entry_get_calib_cb_dim = customtkinter.CTkEntry(
            master=self.sidebar_frame_cam_calib,
            placeholder_text="6x5",
            placeholder_text_color="#808080",
            width=50
        )
        self.sidebar_entry_get_calib_sq_size_label = customtkinter.CTkLabel(
            master=self.sidebar_frame_cam_calib,
            text="Square size (m):"
        )
        self.sidebar_entry_get_calib_sq_size = customtkinter.CTkEntry(
            master=self.sidebar_frame_cam_calib,
            placeholder_text="0.025",
            placeholder_text_color="#808080",
            width=50
        )
        self.sidebar_btn_set_calib_success_label = customtkinter.CTkLabel(
            master=self.sidebar_frame_cam_calib,
            text="",
            text_color='green',
            font=customtkinter.CTkFont(size=25, weight="bold"),

        )
        self.sidebar_btn_set_calib = customtkinter.CTkButton(
            self.sidebar_frame_cam_calib,
            text="Update",
            command=self.sidebar_btn_update_calib_event,
            width=85


        )
        self.sidebar_entry_get_calib_cb_dim_label.grid(
            row=1, column=0, padx=(20, 0),  pady=(10, 5), sticky="nsw")
        self.sidebar_entry_get_calib_sq_size_label.grid(
            row=2, column=0, padx=(20, 0),  pady=0, sticky="nsw")
        self.sidebar_entry_get_calib_cb_dim.grid(
            row=1, column=1, padx=(10, 10), pady=(10, 5), sticky="nsw")
        self.sidebar_entry_get_calib_sq_size.grid(
            row=2, column=1, padx=(10, 10), pady=0, sticky="nsw")
        self.sidebar_btn_set_calib.grid(
            row=3, column=0, padx=(10, 10), pady=(10, 0), sticky='e')
        self.sidebar_btn_set_calib_success_label.grid(
            row=3, column=1, padx=(20, 0), pady=(10, 0), sticky='nsw')
        self.sidebar_btn_cam_1_calib.grid(
            row=4, column=0, padx=10, pady=10, columnspan=2)
        self.sidebar_btn_cam_2_calib.grid(
            row=5, column=0, padx=10, pady=0, columnspan=2)
        self.sidebar_btn_cam_3_calib.grid(
            row=6, column=0, padx=10, pady=(10, 20), columnspan=2)

        self.sidebar_frame_cam_view_label = customtkinter.CTkLabel(
            master=self.sidebar_frame_cam_view,
            text="Camera View"
        )
        self.sidebar_btn_cam_1_start = customtkinter.CTkButton(
            master=self.sidebar_frame_cam_view,
            text='Start Camera 1',
            command=lambda: self.sidebar_camera_btn_event(1, True, False)
        )
        self.sidebar_btn_cam_2_start = customtkinter.CTkButton(
            master=self.sidebar_frame_cam_view,
            text='Start Camera 2',
            command=lambda: self.sidebar_camera_btn_event(2, True, False)
        )
        self.sidebar_btn_cam_3_start = customtkinter.CTkButton(
            master=self.sidebar_frame_cam_view,
            text='Start Camera 3',
            command=lambda: self.sidebar_camera_btn_event(3, True, False)
        )
        self.sidebar_frame_cam_view_label.grid(
            row=0, column=0, columnspan=2, padx=25, pady=(10, 10))
        self.sidebar_btn_cam_1_start.grid(
            row=1, column=0, columnspan=2, padx=25, pady=(0, 10))
        self.sidebar_btn_cam_2_start.grid(
            row=2, column=0, columnspan=2, padx=25, pady=0)
        self.sidebar_btn_cam_3_start.grid(
            row=3, column=0, columnspan=2, padx=25, pady=(10, 20))
        self.appearance_mode_label = customtkinter.CTkLabel(
            master=self.sidebar_frame_ui,
            text="Appearance:"
        )
        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(
            master=self.sidebar_frame_ui,
            values=["Light", "Dark", "System"],
            command=self.change_appearance_mode_event,
            width=40
        )
        # self.ui_color_optionmenu = customtkinter.CTkOptionMenu(
        #     master=self.sidebar_frame_ui,
        #     values=["Blue", "Dark Blue", "Green"],
        #     command=self.change_color_event,
        #     width=40
        # )
        self.sidebar_marker_size_entry = customtkinter.CTkEntry(
            master=self.sidebar_frame_ui,
            placeholder_text="0.1",
            textvariable=self.var_marker_size,
            placeholder_text_color="#808080",
            width=50
        )

        self.sidebar_marker_size_label = customtkinter.CTkLabel(
            master=self.sidebar_frame_ui,
            text="Marker Size (m):"
        )
        self.sidebar_dictionary_label = customtkinter.CTkLabel(
            master=self.sidebar_frame_ui,
            text="Dictionary:"
        )
        self.sidebar_dictionary_entry = customtkinter.CTkEntry(
            master=self.sidebar_frame_ui,
            placeholder_text="1",
            textvariable=self.var_dictionary,
            placeholder_text_color="#808080",
            width=50
        )
        self.sidebar_set_dictionary_btn = customtkinter.CTkButton(
            master=self.sidebar_frame_ui,
            text="Set Dictionary",
            command=self.sidebar_set_dictionary_btn_event
        )
        self.sidebar_dictionary_explain_label = customtkinter.CTkLabel(
            master=self.sidebar_frame_ui,
            text= 'Dictionary values:\n1 = (DICT_4X4_100)\n5  = (DICT_5X5_100)\n9  = (DICT_6X6_100)\n13 = (DICT_7X7_100)\nhttp://wiki.ros.org/aruco_detect')
        
        self.sidebar_marker_size_label.grid(
            row=0, column=0, padx=(16, 0), pady=(10, 5), sticky="nsw")
        self.sidebar_marker_size_entry.grid(
            row=0, column=1, padx=(0, 10), pady=(10, 5), sticky="nse")
        self.sidebar_dictionary_label.grid(
            row=1, column=0, padx=(16, 0), pady=(0, 5), sticky="nsw")
        self.sidebar_dictionary_entry.grid(
            row=1, column=1, padx=(0, 10), pady=(0, 5), sticky="nse")
        self.sidebar_set_dictionary_btn.grid(
            row=2, column=0, columnspan=2, padx=(0, 10), pady=(0, 5), sticky="nse")
        self.appearance_mode_label.grid(
            row=3, column=0, padx=(16, 0), pady=(0, 5), sticky="nsw")
        self.appearance_mode_optionemenu.grid(
            row=3, column=1, padx=(0, 10), pady=(0, 5), sticky="nse")
        # self.sidebar_dictionary_explain_label.grid(
            # row=4, column=0, columnspan=2, padx=(16, 0), pady=(0, 5), sticky="nsw")
        

        self.tabview = customtkinter.CTkTabview(
            master=self,
            fg_color='#C4C3C3'
        )
        self.main_button_exit = customtkinter.CTkButton(
            master=self,
            text='EXIT',
            command=self.exit_button_click,
            fg_color=("gray10", "#DCE4EE"),
            border_width=2,
            text_color=("#DCE4EE", "gray10")
        )
        self.tabview.grid(row=1, column=1, padx=20,
                          pady=(5, 10), sticky="nsew")
        self.main_button_exit.grid(
            row=2, column=1, padx=(20, 20), pady=(0, 20))

        self.tabview.add("Record & Process Data")
        self.tabview.add("Display Results")
        self.tabview.tab("Record & Process Data").grid_columnconfigure(
            0, weight=1)  # configure grid of individual tabs
        self.tabview.tab("Display Results").grid_columnconfigure(
            0, weight=1)  # configure grid of individual tabs

        self.result_label = customtkinter.CTkLabel(
            master=self.tabview.tab("Display Results"),
            text="Results",
            font=customtkinter.CTkFont(size=16),
            text_color='black'
        )
        self.result_frame = customtkinter.CTkFrame(
            master=self.tabview.tab("Display Results"),
            fg_color=('lightgray', 'gray')
        )

        self.result_label.grid(row=0, column=0, padx=20,
                               pady=(5, 0), sticky="nsew")
        self.result_frame.grid(
            row=1, column=0, columnspan=2, padx=20, pady=(10, 0), sticky="nsew")
        self.record_label = customtkinter.CTkLabel(
            master=self.tabview.tab("Record & Process Data"),
            text="Recording from Camera",
            font=customtkinter.CTkFont(size=16),
            text_color='black'
        )
        self.record_single_frame = customtkinter.CTkFrame(
            master=self.tabview.tab("Record & Process Data"),
            fg_color=('lightgray', 'gray')
        )
        self.record_multiple_frame = customtkinter.CTkFrame(
            master=self.tabview.tab("Record & Process Data"),
            fg_color=('lightgray', 'gray')
        )
        self.process_label = customtkinter.CTkLabel(
            master=self.tabview.tab("Record & Process Data"),
            text="Processing Data",
            font=customtkinter.CTkFont(size=16),
            text_color='black'
        )

        self.process_frame = customtkinter.CTkFrame(
            master=self.tabview.tab("Record & Process Data"),
            fg_color=('lightgray', 'gray')
        )
        self.record_label.grid(row=0, column=0, padx=20,
                               pady=(5, 0), sticky="nsew")
        self.record_single_frame.grid(
            row=1, column=0, padx=20, pady=(10, 0), sticky="nsew")
        self.record_multiple_frame.grid(
            row=2, column=0, padx=20, pady=(20, 0), sticky="nsew")
        self.process_label.grid(row=3, column=0, padx=20,
                                pady=10,     sticky="nsew")
        self.process_frame.grid(row=4, column=0, padx=20,
                                pady=(0, 20), sticky="nsew")
        # self.record_multiple_frame.rowconfigure(0, weight=1)

        self.process_load_last_saved_button = customtkinter.CTkButton(
            master=self.process_frame,
            text="Load Current File",
            font=customtkinter.CTkFont(size=14),
            command=self.load_last_saved_button_event
        )
        self.process_load_options_label = customtkinter.CTkLabel(
            master=self.process_frame,
            text="Load File from Directory:",
            font=customtkinter.CTkFont(size=14)
        )
        self.process_load_button = customtkinter.CTkButton(
            master=self.process_frame,
            text="Open File",
            font=customtkinter.CTkFont(size=14),
            command=self.load_data_button_event
        )
        self.process_detect_button = customtkinter.CTkButton(
            master=self.process_frame,
            text="Start Post-Processing",
            font=customtkinter.CTkFont(size=14),
            command=self.detect_button_event
        )
        self.process_detect_label = customtkinter.CTkLabel(
            master=self.process_frame,
            text="Completed Successfully",
            state="disabled"
        )
        self.process_show_results_button = customtkinter.CTkButton(
            master=self.process_frame,
            text="Show Results",
            state = 'disabled',
            font=customtkinter.CTkFont(size=14),
            command=self.show_results_button_event
        )

        self.process_load_last_saved_button.grid(
            row=0, column=0, padx=10, pady=10,   sticky="nsew")
        self.process_load_options_label.grid(
            row=0, column=1, padx=10, pady=10,   sticky="nsew")
        self.process_load_button.grid(
            row=0, column=2, padx=10, pady=10,   sticky="nsew")
        self.process_detect_button.grid(
            row=1, column=1, padx=10, pady=10,   sticky="nsew")
        self.process_detect_label.grid(
            row=1, column=2, padx=10, pady=10,   sticky="nsew")
        self.process_show_results_button.grid(
            row=1, column=3, padx=10, pady=10,   sticky="nsew")

        self.single_cam_label = customtkinter.CTkLabel(
            master=self.record_single_frame,
            text="Single camera",
            font=customtkinter.CTkFont(size=16),
            text_color="black"
        )
        self.single_cam_select_label = customtkinter.CTkLabel(
            master=self.record_single_frame,
            text="Select a Camera:",
            font=customtkinter.CTkFont(size=14)
            # text_color="#707070"
        )
        self.single_rec_manual_label = customtkinter.CTkLabel(
            master=self.record_single_frame,
            text="Start Recording manually:",
            font=customtkinter.CTkFont(size=14)
            # text_color="#707070"
        )
        self.single_dur_select_or_label = customtkinter.CTkLabel(
            master=self.record_single_frame,
            text="or        Enter manually:",
            font=customtkinter.CTkFont(size=14)
            # text_color="#707070"
        )
        self.single_camera_1_radio = customtkinter.CTkRadioButton(
            master=self.record_single_frame,
            text="Camera 1",
            font=customtkinter.CTkFont(size=14),
            # command=self.single_camera_radio_event,
            variable=self.camera_selection_var,
            value="Camera 1"
            # text_color="#808080"
        )
        self.single_camera_2_radio = customtkinter.CTkRadioButton(
            master=self.record_single_frame,
            text="Camera 2",
            font=customtkinter.CTkFont(size=14),
            # command=self.single_camera_radio_event,
            variable=self.camera_selection_var,
            value="Camera 2"
            # text_color="#808080"
        )
        self.single_camera_3_radio = customtkinter.CTkRadioButton(
            master=self.record_single_frame,
            text="Camera 3",
            font=customtkinter.CTkFont(size=14),
            # command=self.single_camera_radio_event,
            variable=self.camera_selection_var,
            value="Camera 3"
            # text_color="#808080"
        )
        self.single_dur_select_label = customtkinter.CTkLabel(
            master=self.record_single_frame,
            text="Select duration (s):",
            font=customtkinter.CTkFont(size=14)
        )
        self.single_camera_dur_combo_box = customtkinter.CTkComboBox(
            master=self.record_single_frame,
            values=['10',
                    '20',
                    '30',
                    '40',
                    '50',
                    '60'
                    ],
            variable=self.single_camera_duration_var
        )
        self.single_camera_dur_entry = customtkinter.CTkEntry(
            master=self.record_single_frame,
            placeholder_text="Enter",
            justify='center',
            # placeholder_text_color="#808080",
            corner_radius=5,
            width=20
        )
        self.single_camera_rec_button = customtkinter.CTkButton(
            master=self.record_single_frame,
            text="Record",
            font=customtkinter.CTkFont(size=14),
            command=self.record_single_camera
        )
        self.single_camera_rec_manual_button = customtkinter.CTkButton(
            master=self.record_single_frame,
            text="Record",
            font=customtkinter.CTkFont(size=14),
            fg_color=("#DCE4EE", "gray10"),
            border_width=2,
            text_color=("gray10", "#DCE4EE")
        )

        self.single_cam_label.grid(row=0, column=1, padx=10, pady=(
            10, 10), sticky="nsew", columnspan=2, rowspan=1)
        self.single_cam_select_label.grid(
            row=1, column=0, padx=10, pady=(5, 5),   sticky="nsew")
        self.single_camera_1_radio.grid(
            row=1, column=1, padx=5,  pady=(0, 5),   sticky="nsew")
        self.single_camera_2_radio.grid(
            row=1, column=2, padx=5,  pady=(0, 5),   sticky="nsew")
        self.single_camera_3_radio.grid(
            row=1, column=3, padx=5,  pady=(0, 5),   sticky="nsew")
        self.single_dur_select_label.grid(
            row=2, column=0, padx=(20, 5),  pady=(0, 5),   sticky="nsew")
        self.single_camera_dur_combo_box.grid(
            row=2, column=1, padx=0,  pady=(0, 5),   sticky="nsew")
        self.single_dur_select_or_label.grid(
            row=2, column=2, padx=(0, 10),  pady=(0, 5),   sticky="nse")
        self.single_camera_dur_entry.grid(
            row=2, column=3, padx=(0, 20),  pady=(0, 5),   sticky="nsew")
        self.single_camera_rec_button.grid(
            row=3, column=1, padx=(10), pady=(10, 20), sticky="nsew")
        self.single_rec_manual_label.grid(
            row=3, column=2, padx=(0, 10), pady=(10, 20), sticky="nsew")
        self.single_camera_rec_manual_button.grid(
            row=3, column=3, padx=(0, 20),  pady=(10, 20), sticky="nsew")
# *****************************************************************************************************************************

        self.multiple_cams_label = customtkinter.CTkLabel(
            master=self.record_multiple_frame,
            text="Multiple cameras",
            font=customtkinter.CTkFont(size=16),
            # text_color="#505050"
        )
        self.multi_cam_select_label = customtkinter.CTkLabel(
            master=self.record_multiple_frame,
            text="Select Cameras:",
            font=customtkinter.CTkFont(size=14),
            # text_color="#707070"
        )
        self.multi_dur_select_label = customtkinter.CTkLabel(
            master=self.record_multiple_frame,
            text="Select duration (s):",
            font=customtkinter.CTkFont(size=14),
            # text_color="#707070"
        )
        self.multi_rec_manual_label = customtkinter.CTkLabel(
            master=self.record_multiple_frame,
            text="Start Recording manually:",
            font=customtkinter.CTkFont(size=14),
            # text_color="#707070"
        )
        self.multi_dur_select_or_label = customtkinter.CTkLabel(
            master=self.record_multiple_frame,
            text="or        Enter manually:",
            font=customtkinter.CTkFont(size=14),
            # text_color="#707070"
        )

        self.multi_camera_1_checkbox = customtkinter.CTkCheckBox(
            master=self.record_multiple_frame,
            text="Camera 1",
            font=customtkinter.CTkFont(size=14),
            checkbox_width=20,
            checkbox_height=20,
            command=self.check_camera_1_checkbox_event,
            variable=self.check_camera_1_var,
            onvalue="on",
            offvalue="off",
            # text_color="#808080"

        )
        self.multi_camera_2_checkbox = customtkinter.CTkCheckBox(
            master=self.record_multiple_frame,
            text="Camera 2",
            font=customtkinter.CTkFont(size=14),
            checkbox_width=20,
            checkbox_height=20,
            command=self.check_camera_2_checkbox_event,
            variable=self.check_camera_2_var,
            onvalue="on",
            offvalue="off",
            # text_color="#808080"

        )
        self.multi_camera_3_checkbox = customtkinter.CTkCheckBox(
            master=self.record_multiple_frame,
            text="Camera 3",
            font=customtkinter.CTkFont(size=14),
            checkbox_width=20,
            checkbox_height=20,
            command=self.check_camera_3_checkbox_event,
            variable=self.check_camera_3_var,
            onvalue="on",
            offvalue="off",
            # text_color="#808080"
        )
        self.multi_camera_dur_combo_box = customtkinter.CTkComboBox(
            master=self.record_multiple_frame,
            values=['10',
                    '20',
                    '30',
                    '40',
                    '50',
                    '60'
                    ],
            variable=self.multi_camera_duration_var
        )
        self.multi_camera_dur_entry = customtkinter.CTkEntry(
            master=self.record_multiple_frame,
            placeholder_text="Enter",
            justify="center",
            # placeholder_text_color="#808080",
            corner_radius=5,
            width=20
        )
        self.multi_camera_start_button = customtkinter.CTkButton(
            master=self.record_multiple_frame,
            text="Start Cameras",
            font=customtkinter.CTkFont(size=14),
            command=self.start_multi_camera
        )
        self.multi_camera_record_button = customtkinter.CTkButton(
            master=self.record_multiple_frame,
            text="Record",
            font=customtkinter.CTkFont(size=14),
            command=self.record_multi_camera
        )
        self.multi_camera_rec_manual_button = customtkinter.CTkButton(
            master=self.record_multiple_frame,
            text="Record",
            font=customtkinter.CTkFont(size=14),
            fg_color=("#DCE4EE", "gray10"),
            border_width=2,
            text_color=("gray10", "#DCE4EE")
        )

        self.multiple_cams_label.grid(row=0, column=1, padx=10, pady=(
            10, 10), sticky="nsew", columnspan=2, rowspan=1)
        self.multi_cam_select_label.grid(
            row=1, column=0, padx=10, pady=(5, 5),   sticky="nsew")
        self.multi_camera_1_checkbox.grid(
            row=1, column=1, padx=5,  pady=(0, 5),   sticky="nsew")
        self.multi_camera_2_checkbox.grid(
            row=1, column=2, padx=5,  pady=(0, 5),   sticky="nsew")
        self.multi_camera_3_checkbox.grid(
            row=1, column=3, padx=5,  pady=(0, 5),   sticky="nsew")
        self.multi_dur_select_label.grid(
            row=2, column=0, padx=(20, 5),  pady=(0, 5),   sticky="nsew")
        self.multi_camera_dur_combo_box.grid(
            row=2, column=1, padx=0,  pady=(0, 5),   sticky="nsew")
        self.multi_dur_select_or_label.grid(
            row=2, column=2, padx=(0, 10),  pady=(0, 5),   sticky="nsew")
        self.multi_camera_dur_entry.grid(
            row=2, column=3, padx=(0, 20),  pady=(0, 5),   sticky="nsew")
        self.multi_camera_start_button.grid(
            row=3, column=0, padx=10, pady=(10, 20),  sticky="nsew")
        self.multi_camera_record_button.grid(
            row=3, column=1, padx=10, pady=(10, 20),  sticky="nsew")
        self.multi_rec_manual_label.grid(
            row=3, column=2, padx=(0, 10), pady=(10, 20), sticky="nsew")
        self.multi_camera_rec_manual_button.grid(
            row=3, column=3, padx=(0, 20),  pady=(10, 20), sticky="nsew")
    def record_multi_camera(self):
        print('\033[92m***************************************************')
        print('********  Recording multi camera bag file  ********')
        print('***************************************************')
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.record_multibag_launch_file = roslaunch.parent.ROSLaunchParent(
            self.uuid, [self.record_multibag_launch_path])
        # self.record_multibag_launch_file.start()
        return
        
        if self.sidebar_marker_size_entry.get() != "":
            self.maker_size = self.sidebar_marker_size_entry.get()
        print('\033[93mmaker_size:', self.maker_size)
        multi_manual_dur = self.multi_camera_dur_entry.get()
        multi_combo_dur = self.multi_camera_dur_combo_box.get()
        
        # print('multi_manual_dur', multi_manual_dur)
        # print('multi_combo_dur', multi_combo_dur)
        if multi_manual_dur == "":
            if multi_combo_dur == "Select":
                print("Please select a duration")
                return
            else:
                self.multi_camera_dur = multi_combo_dur
                print(f"\033[93mDuration: {self.multi_camera_dur}", "\033[0m")
                print('******************************************')
        else:
            self.multi_camera_dur = multi_manual_dur
            print(f"\033[93mDuration: {self.multi_camera_dur}", "\033[0m")
            print('******************************************')


    def check_camera_1_checkbox_event(self):
        print(self.check_camera_1_var.get())
        pass
        
    def check_camera_2_checkbox_event(self):
        print(self.check_camera_2_var.get())
        pass
    def check_camera_3_checkbox_event(self):
        print(self.check_camera_3_var.get())
        pass

    def record_data(self):
        print('Recording data')
        print(self.optionmenu_1.get())
        print(self.combobox_1.get())

    def open_input_dialog_event(self):
        dialog = customtkinter.CTkInputDialog(
            text="Type in a number:", title="CTkInputDialog")
        print("CTkInputDialog:", dialog.get_input())

    def change_appearance_mode_event(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)

    def change_color_event(self, new_color_string: str):
        pass
        # customtkinter.set_default_color_theme(new_color_string)
        # customtkinter.set_default_color_theme(val)
        # new_scaling_float = int(new_scaling.replace("%", "")) / 100
        # customtkinter.set_widget_scaling(new_scaling_float)

    def start_camera(self, camera_name, device_id, calibration_file, view_camera, calibrate_camera):
        """Starts a camera driver and optionally a camera view"""
        camera_launch_args = [f"{self.cam_launch}",
                              f"cam:={camera_name}",
                              f"device_id:={device_id}",
                              f"calib_file:={calibration_file}"
                              ]
        view_launch_args = [self.view_launch,
                            f"camera_name:={camera_name}"
                            ]
        # print(f"board size: {self.board_size} and square size: {self.square_size}")
        # calib_launch_args = [self.calib_launch,
        #                      f"cam:={camera_name}",
        #                      f"size:={self.board_size}",
        #                      f"square:={self.square_size}"
        #                      ]

        # Create a ROS launch file with the camera launch command
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(
            camera_launch_args)[0], camera_launch_args[1:])]
        cam_driver = roslaunch.parent.ROSLaunchParent(
            self.uuid, roslaunch_file)

        # Start the camera driver
        try:
            cam_driver.start()
            self.running_processes[f'{camera_name}_driver'] = cam_driver

            # Set camera active flag to True
            if camera_name == 'camera_1':
                self.camera_1_active = True
            elif camera_name == 'camera_2':
                self.camera_2_active = True
            elif camera_name == 'camera_3':
                self.camera_3_active = True

            # Print success message
            rospy.loginfo(f"{camera_name} camera driver started successfully.")
            rospy.sleep(2)

            # If view_camera is True, start the camera view
            if view_camera:
                view_launch_file = [(roslaunch.rlutil.resolve_launch_arguments(
                    view_launch_args)[0], view_launch_args[1:])]
                view_output = roslaunch.parent.ROSLaunchParent(
                    self.uuid, view_launch_file)
                view_output.start()
                self.running_processes[f'{camera_name}_view'] = view_output

            # If calibrate_camera is True, start camera calibration
            # if calibrate_camera:
            #     rospy.loginfo(
            #         f"{camera_name} calibration started successfully.")
            #     calibrate_launch_file = [(roslaunch.rlutil.resolve_launch_arguments(
            #         calib_launch_args)[0], calib_launch_args[1:])]
            #     camera_calibrate = roslaunch.parent.ROSLaunchParent(
            #         self.uuid, calibrate_launch_file)
            #     camera_calibrate.start()
            #     self.running_processes[f'{camera_name}_calibrate'] = camera_calibrate

        except roslaunch.RLException as excep_camera:
            rospy.logerr(
                f"Error starting {camera_name} camera driver: {str(excep_camera)}")
            return

    def stop_camera(self, camera_name):
        """Stop a camera driver."""
        # Check if camera driver is running
        if f'{camera_name}_driver' not in self.running_processes:
            rospy.logwarn(f"{camera_name} camera driver is not running.")
            return

        # Shutdown the camera driver
        try:
            self.running_processes[f'{camera_name}_driver'].shutdown()
            if f'{camera_name}_view' in self.running_processes:
                self.running_processes[f'{camera_name}_view'].shutdown()
            # if f'{camera_name}_calibrate' in self.running_processes:
            #     self.running_processes[f'{camera_name}_calibrate'].shutdown()
        except roslaunch.RLException as excep_camera:
            rospy.logerr(
                f"Error stopping {camera_name} camera driver: {str(excep_camera)}")
            return

        # Set camera active flag to False
        if camera_name == 'camera_1':
            self.camera_1_active = False
        elif camera_name == 'camera_2':
            self.camera_2_active = False
        elif camera_name == 'camera_3':
            self.camera_3_active = False

        # Remove camera driver from running processes dictionary
        self.running_processes.pop(f'{camera_name}_driver', None)
        self.running_processes.pop(f'{camera_name}_view', None)
        # self.running_processes.pop(f'{camera_name}_calibrate', None)
        # Print success message

        rospy.loginfo(f"{camera_name} camera driver stopped successfully.")

    def sidebar_camera_btn_event(self, camera_number, show_camera, calibrate_camera):
        """This function is called when a camera button is pressed"""
        cameras = [
            {'camera_name': 'camera_1', 'device_id': 0, 'calibration_file': 'cam1', 'button': self.sidebar_btn_cam_1_start,
                'calibrate_button': self.sidebar_btn_cam_1_calib, 'name': 'Camera 1'},
            {'camera_name': 'camera_2', 'device_id': 1, 'calibration_file': 'cam2', 'button': self.sidebar_btn_cam_2_start,
                'calibrate_button': self.sidebar_btn_cam_2_calib, 'name': 'Camera 2'},
            {'camera_name': 'camera_3', 'device_id': 2, 'calibration_file': 'cam3',
                'button': self.sidebar_btn_cam_3_start, 'calibrate_button': self.sidebar_btn_cam_3_calib, 'name': 'Camera 3'}
        ]

        # Select the camera based on the provided number
        if camera_number < 1 or camera_number > 3:
            rospy.logerr(f"Invalid camera number: {camera_number}")
            return

        camera = cameras[camera_number - 1]

        camera_name = camera['camera_name']
        device_id = camera['device_id']
        calibration_file = camera['calibration_file']
        button = camera['calibrate_button'] if calibrate_camera else camera['button']
        button_name = 'Calibrate' if calibrate_camera else 'Start'
        # Get the current state of the camera
        camera_active_states = [self.camera_1_active,
                                self.camera_2_active,
                                self.camera_3_active]
        camera_active = camera_active_states[camera_number - 1]

        # Start or stop the camera depending on its current state
        try:
            if not camera_active:
                # Start the selected camera
                self.start_camera(camera_name, device_id,
                                  calibration_file, show_camera, calibrate_camera)

                # Update button text and color
                button.configure(
                    text=f"Stop {camera['name']}", fg_color=("#fa5f5a", "#ba3732"))

                # Set the camera active flag to True
                camera_active_states[camera_number - 1] = True
            else:
                # Stop the selected camera
                self.stop_camera(camera_name)

                # Update button text and color
                button.configure(
                    text=f"{button_name} {camera['name']}", fg_color=themes[COLOR_SELECT])

                # Set the camera active flag to False
                camera_active_states[camera_number - 1] = False
        except Exception as excep_camera:
            rospy.logerr(
                f"Error {'' if camera_active else 'starting'} {camera_name} camera: {str(excep_camera)}")
            return

        # Update the camera active states
        self.camera_1_active, self.camera_2_active, self.camera_3_active = camera_active_states

    def sidebar_btn_update_calib_event(self):
        """This function is called when the set calibration button is pressed and updates the calibration values"""
        self.board_size = self.sidebar_entry_get_calib_cb_dim.get(
        ) if self.sidebar_entry_get_calib_cb_dim.get() != '' else self.board_size
        self.square_size = self.sidebar_entry_get_calib_sq_size.get(
        ) if self.sidebar_entry_get_calib_sq_size.get() != '' else self.square_size
        if self.board_size == '6x5' and self.square_size == '0.025':
            rospy.logerr('Please enter new calibtration parameters!')
        else:
            rospy.loginfo('Checkerboard parameters updated successfully')
            self.sidebar_btn_set_calib_success_label.configure(text="☑")
            print('board size', self.board_size)
            print('square size', self.square_size)

    def start_camera_record(self, camera_name, device_id, calibration_file, dur):
        """Starts a camera driver and optionally a camera view"""
        # print('entered camera record function')
        time_dur_bag = dur
        camera_num = device_id + 1
        self.recorded_datetime_var = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        # bagfile_name = f"cam{camera_num}_bagfile"
        # LAUNCHFILE PARAMETERS FOR CAMERA AND BAG RECORDING
        camera_launch_args = [f"{self.cam_launch}",
                              f"cam:={camera_name}",
                              f"device_id:={device_id}",
                              f"calib_file:={calibration_file}"
                              ]
        record_launch_args = [
            self.record_bag_launch,
            f'cam:=camera_{camera_num}',
            f'dur:={time_dur_bag}',
            f'bagfile_datetime:={self.recorded_datetime_var}'
        ]
        # Create a ROS launch file with the camera launch command
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(
            camera_launch_args)[0], camera_launch_args[1:])]
        cam_driver = roslaunch.parent.ROSLaunchParent(
            self.uuid, roslaunch_file)

        # Create a ROS launch file with the camera record launch command
        record_single_cam_file = [(roslaunch.rlutil.resolve_launch_arguments(
            record_launch_args)[0], record_launch_args[1:])]
        record_single_cam = roslaunch.parent.ROSLaunchParent(
            self.uuid, record_single_cam_file)

        # Start the camera driver
        try:
            cam_driver.start()
            self.running_processes[f'{camera_name}_driver'] = cam_driver
            # print('camera driver has successfully started')

            # Set camera active flag to True
            if camera_name == 'camera_1':
                self.camera_1_active = True
            elif camera_name == 'camera_2':
                self.camera_2_active = True
            elif camera_name == 'camera_3':
                self.camera_3_active = True

            # Print success message
            rospy.loginfo(f"{camera_name} camera driver started successfully.")
            rospy.sleep(1)
            print('**********************************hello *********************************')
             
            record_single_cam.start()
            self.running_processes[f'{camera_name}_record'] = record_single_cam
            rec_time = 0
            while record_single_cam.pm.is_alive():
                rospy.sleep(1)
                rec_time += 1
                if rec_time <= int(time_dur_bag):
                    print(
                        f"\033[93mRecording from {camera_name}...{rec_time}/{time_dur_bag}s\033[0m")

            # Shutdown the camera driver
            try:
                self.running_processes[f'{camera_name}_driver'].shutdown()
                if f'{camera_name}_record' in self.running_processes:
                    self.running_processes[f'{camera_name}_record'].shutdown()

                    saved_bagfile = f"{camera_name}_{time_dur_bag}s_{self.recorded_datetime_var}.bag"
                    self.last_recorded_bag_file_name_with_path = os.path.join(
                        self.bagfile_path, saved_bagfile)
                    print("\033[93mSaved Data: \033[0m")
                    print("\033[93m------------\033[0m")
                    print(
                        f"\033[93mBagfile Name: {os.path.basename(self.last_recorded_bag_file_name_with_path)} \033[0m")
                    print(
                        f"\033[93mDirectory: {os.path.dirname(self.last_recorded_bag_file_name_with_path)} \033[0m")

            except roslaunch.RLException as excep_camera:
                rospy.logerr(
                    f"Error stopping {camera_name} camera driver: {str(excep_camera)}")
                return

            # Set camera active flag to False
            if camera_name == 'camera_1':
                self.camera_1_active = False
            elif camera_name == 'camera_2':
                self.camera_2_active = False
            elif camera_name == 'camera_3':
                self.camera_3_active = False

            # Remove camera driver from running processes dictionary
            self.running_processes.pop(f'{camera_name}_driver', None)
            self.running_processes.pop(f'{camera_name}_record', None)

            print(f"\033[92m\nSuccessfully saved bag file from {camera_name}!")
            print(
                '***************************************************\n\033[0m')

        except roslaunch.RLException as excep_camera:
            rospy.logerr(
                f"Error starting {camera_name} camera driver: {str(excep_camera)}")
            return

    def stop_camera_record(self, camera_name):
        """Stop a camera driver."""
        # Check if camera driver is running
        if f'{camera_name}_driver' not in self.running_processes:
            rospy.logwarn(f"{camera_name} camera driver is not running.")
            return

        # Shutdown the camera driver
        try:
            self.running_processes[f'{camera_name}_driver'].shutdown()
            if f'{camera_name}_record' in self.running_processes:
                self.running_processes[f'{camera_name}_record'].shutdown()

        except roslaunch.RLException as excep_camera:
            rospy.logerr(
                f"Error stopping {camera_name} camera driver: {str(excep_camera)}")
            return

        # Set camera active flag to False
        if camera_name == 'camera_1':
            self.camera_1_active = False
        elif camera_name == 'camera_2':
            self.camera_2_active = False
        elif camera_name == 'camera_3':
            self.camera_3_active = False

        # Remove camera driver from running processes dictionary
        self.running_processes.pop(f'{camera_name}_driver', None)
        self.running_processes.pop(f'{camera_name}_record', None)

        rospy.loginfo(f"{camera_name} camera bag file saved successfully.")

    def record_single_camera(self):
        """This function is called when the record single camera button is pressed"""
        if self.sidebar_marker_size_entry.get() != "":
            self.maker_size = self.sidebar_marker_size_entry.get()
        print('maker_size:', self.maker_size)

        cameras = [
            {'camera_name': 'camera_1', 'device_id': 0,
                'calibration_file': 'cam1', 'button': self.sidebar_btn_cam_1_start},
            {'camera_name': 'camera_2', 'device_id': 1,
                'calibration_file': 'cam2', 'button': self.sidebar_btn_cam_2_start},
            {'camera_name': 'camera_3', 'device_id': 2,
                'calibration_file': 'cam3', 'button': self.sidebar_btn_cam_3_start}
        ]
        camera_selected = self.camera_selection_var.get()
        if camera_selected == "Camera 1":
            camera_number = 1
        elif camera_selected == "Camera 2":
            camera_number = 2
        elif camera_selected == "Camera 3":
            camera_number = 3
        else:
            print("Please select a camera")
            return

        man_dur = self.single_camera_dur_entry.get()
        combo_dur = self.single_camera_dur_combo_box.get()
        print()
        print("\033[92m*********************************************\033[0m")
        print("\033[92m******  Saving single camera bag file  ******\033[0m")
        print("\033[92m*********************************************\033[0m")
        print()
        print("\033[93mCamera Selected: ", camera_selected, "\033[0m")

        if man_dur == "":
            if combo_dur == "Select":
                print("Please select a duration")
                return
            else:
                self.single_camera_dur = combo_dur
                print(f"\033[93mDuration: {self.single_camera_dur}", "\033[0m")
                print('******************************************')
        else:
            self.single_camera_dur = man_dur
            print(f"\033[93mDuration: {self.single_camera_dur}", "\033[0m")
            print('******************************************')

        # time.sleep(1)

        # Select the camera based on the provided number
        if camera_number < 1 or camera_number > 3:
            rospy.logerr(f"Invalid camera number: {camera_number}")
            return

        camera = cameras[camera_number - 1]

        camera_name = camera['camera_name']
        device_id = camera['device_id']
        calibration_file = camera['calibration_file']

        button = self.single_camera_rec_button
        # button_name = 'Calibrate' if calibrate_camera else 'Start'

        # Get the current state of the camera
        camera_active_states = [self.camera_1_active,
                                self.camera_2_active, self.camera_3_active]
        camera_active = camera_active_states[camera_number - 1]

        # Start or stop the camera depending on its current state
        try:
            if not camera_active:

                # Update button color
                button.configure(fg_color=("#fa5f5a", "#ba3732"))

                # Set the camera active flag to True
                camera_active_states[camera_number - 1] = True

                # Start the selected camera
                self.start_camera_record(
                    camera_name, device_id, calibration_file, self.single_camera_dur)

                # Update button color
                button.configure(fg_color=themes[COLOR_SELECT])

                # Set the camera active flag to False
                camera_active_states[camera_number - 1] = False

        except Exception as excep_camera:
            rospy.logerr(
                f"Error {'' if camera_active else 'starting'} {camera_name} camera: {str(excep_camera)}")
            return

        # Update the camera active states
        self.camera_1_active, self.camera_2_active, self.camera_3_active = camera_active_states


    def start_multi_camera(self):
        print('\033[92m***************************************************')
        print('********  Saving multi camera bag file  ********')
        print('***************************************************')
        # if self.sidebar_marker_size_entry.get() != "":
        #     self.maker_size = self.sidebar_marker_size_entry.get()
        # print('maker_size:', self.maker_size)
        # multi_manual_dur = self.multi_camera_dur_entry.get()
        # multi_combo_dur = self.multi_camera_dur_combo_box.get()
        
        # # print('multi_manual_dur', multi_manual_dur)
        # # print('multi_combo_dur', multi_combo_dur)
        # if multi_manual_dur == "":
        #     if multi_combo_dur == "Select":
        #         print("Please select a duration")
        #         return
        #     else:
        #         self.multi_camera_dur = multi_combo_dur
        #         print(f"\033[93mDuration: {self.multi_camera_dur}", "\033[0m")
        #         print('******************************************')
        # else:
        #     self.multi_camera_dur = multi_manual_dur
        #     print(f"\033[93mDuration: {self.multi_camera_dur}", "\033[0m")
        #     print('******************************************')
            
        cameras = [
            {'camera_name': 'camera_1', 'device_id': 0,
                'calibration_file': 'cam1', 'button': self.sidebar_btn_cam_1_start},
            {'camera_name': 'camera_2', 'device_id': 1,
                'calibration_file': 'cam2', 'button': self.sidebar_btn_cam_2_start},
            {'camera_name': 'camera_3', 'device_id': 2,
                'calibration_file': 'cam3', 'button': self.sidebar_btn_cam_3_start}
        ]
        if self.check_camera_1_var.get() == 'on':
            self.multi_cameras_active_status.add(1)
        else:
            self.multi_cameras_active_status.discard(1)
        if self.check_camera_2_var.get() == 'on':
            self.multi_cameras_active_status.add(2)
        else:
            self.multi_cameras_active_status.discard(2)
        if self.check_camera_3_var.get() == 'on':
            self.multi_cameras_active_status.add(3)
        else:
            self.multi_cameras_active_status.discard(3)

        active_cameras = [cameras[i-1] for i in self.multi_cameras_active_status]
        # print('Active cameras: ', active_cameras)
        
        for camera in active_cameras:
            package = 'gige_cam_driver'
            executable = 'camera_node.py'
            node_name = camera['camera_name']
            device_id = str(camera['device_id'])
            calibration_file = camera['calibration_file']
            calib_path = "$(find gige_cam_driver)/config/camera_info_"+calibration_file+".yaml"
        
            node = roslaunch.core.Node(package, executable, name=node_name, output='screen')
            node.args = f"device_id:={device_id} camera_manager:={node_name} calibration_file:={calib_path}"
            
            launch = roslaunch.scriptapi.ROSLaunch()
            launch.start()

            process = launch.launch(node)
            print("Started camera:", node_name)


        
    
    def load_data_button_event(self):
        """This function is called when the load data button is clicked."""
        print('\033[92m***************************************************')
        print('********  Loading Bag File from Directory  ********')
        print('***************************************************')

        root = tk.Tk()
        root.withdraw()
        self.opened_bagfile_var = filedialog.askopenfilename(
            initialdir=self.bagfile_path)
        try:
            bagfile_name = os.path.basename(self.opened_bagfile_var)
        except Exception as error_load:
            rospy.logerr(f"No bag file selected, {error_load}")
            return
        bagfile_dir = os.path.dirname(self.opened_bagfile_var)
        parts = bagfile_name.split('_')
        # Find the part of the bagfile_name that contains the duration value
        for i, part in enumerate(parts):
            if part.endswith('s'):
                duration_part = part
                break

        if self.opened_bagfile_var != "":

            print('\033[93m')
            print(f"Camera: {parts[0].title()} {parts[1]}")
            print(f"Duration: {duration_part}")
            print(f"Marker Size: {self.var_marker_size}")
            print(f"Recorded Time: {parts[3]}_{parts[-1].split('.')[0]}")
            print(f"Bagfile Name: {bagfile_name}")
            print(f"Directory: {bagfile_dir}")
            print("\033[92m\nSuccessfully loaded bag file from directory!")
            print(
                '***************************************************\n\033[0m')
            self.opened_file_flag = True

    def get_bagfile(self):
        """This function is called to load bagfile."""
        if self.opened_bagfile_var == "" and self.last_recorded_bag_file_name_with_path == "":
            rospy.logerr("No file selected, Please load a bag file first")
            print(
                '\033[92m**********************************************************\n\033[0m')
            return None
        elif self.opened_bagfile_var != "" and self.last_recorded_bag_file_name_with_path == "":
            loaded_file = self.opened_bagfile_var
            print('Opening Bagfile from directory')
            return loaded_file
        elif self.opened_bagfile_var == "" and self.last_recorded_bag_file_name_with_path != "":
            loaded_file = self.last_recorded_bag_file_name_with_path
            print('Loading Bagfile from last recorded file')
            return loaded_file
        else:
            loaded_file = self.opened_bagfile_var
            # loaded_file = self.last_recorded_bag_file_name_with_path
            print("found both files.. but opening from the directory")
            return loaded_file

    def launch_marker_detector(self, camera_name):
        """This function is called to launch the marker detection node."""
        cli_args = [self.detect_launch, f'camera:={camera_name}']
        roslaunch_file = [
            (roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        return roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)

    def detection(self, camera_name):
        """This function is called to launch the marker detection node."""
        print('bagfile received: ', camera_name)
        print('now starting marker detection..')
        if camera_name == 'camera_1':
            marker_detect_camera_1 = self.launch_marker_detector('camera_1')
            marker_detect_camera_1.start()
            self.running_processes.update(marker_detect_camera_1)
            time.sleep(1)
            print('camera_1 marker detection launched')
        elif camera_name == 'camera_2':
            marker_detect_camera_2 = self.launch_marker_detector('camera_2')
            marker_detect_camera_2.start()
            self.running_processes.update(marker_detect_camera_2)
            time.sleep(1)
            print('camera_2 marker detection launched')
        elif camera_name == 'camera_3':
            marker_detect_camera_3 = self.launch_marker_detector('camera_3')
            marker_detect_camera_3.start()
            self.running_processes.update(marker_detect_camera_3)
            print('camera_3 marker detection launched')

    def get_camera_name(self, string_cam):
        try:
            camera_name = "_".join(string_cam.split('/')[-1].split('_')[:2])
            complete_filename = string_cam.split('/')[-1].split('.')[0]
            return [camera_name, complete_filename]
        except:
            return None

    def detect_button_event(self):
        """This function is called when the detect button is clicked."""
        print('\033[92m**********************************************************')
        print('******* Starting Post-Processing from Selected File ******')
        print(
            '**********************************************************\033[93m')
        print()
        bag_play_rate = 2

        filename_ = self.get_bagfile()
        # print(filename_)
        [camera_name, complete_filename] = self.get_camera_name(filename_)
        if camera_name is not None:

            just_filename = complete_filename
            print("Loaded BagFile: ",complete_filename)
            csv_file_path = os.path.join(
                self.csv_file_path, just_filename + '.csv')
            topic_name = '/fiducial_transforms'
            # rostopic_echo_command = f'rostopic echo -p {topic_name} > {csv_file_path}'
            print(just_filename)
            self.sidebar_set_dictionary_btn_event()
            if filename_ is not None:
                readbag_cli_args = [
                    self.read_bag_launch, f"bag_file_path:={filename_}", f"playback_rate:={bag_play_rate}"]
                roslaunch_args = readbag_cli_args[1:]
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(
                    readbag_cli_args)[0], roslaunch_args)]
                rosbag_reading = roslaunch.parent.ROSLaunchParent(
                    self.uuid, roslaunch_file)

                aruco_detect_cli_args = [
                    self.detect_launch, f'camera:={camera_name}', f'dictionary:={self.var_dictionary}', f'aruco_marker_size:={self.var_marker_size}']
                roslaunch_args = aruco_detect_cli_args[1:]
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(
                    aruco_detect_cli_args)[0], roslaunch_args)]
                marker_detection = roslaunch.parent.ROSLaunchParent(
                    self.uuid, roslaunch_file)
                # Start the roslaunch parent object

                try:
                    rosbag_reading.start()
                    self.running_processes[f"{camera_name}_rosbag_reading"] = rosbag_reading
                    print(
                        f'\033[93mPlaying bag file at {bag_play_rate}x speed..\033[0m')
                    try:
                        marker_detection.start()
                        print('\033[93mStarted marker detection..\033[0m')
                        self.running_processes[f"{camera_name}_marker_detection"] = marker_detection
                        print('\033[93mSaving data into a CSV file..\033[0m')
                        with open(csv_file_path, 'w') as f:
                            rostopic_process = subprocess.Popen(
                                ['rostopic', 'echo', '-p', topic_name], stdout=f)
                        # rostopic_process = subprocess.Popen(rostopic_echo_command.split(), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                        # self.running_processes[f"{camera_name}_rostopic_process"] = rostopic_process
                        print('\033[93mFinished reading rosbag file..\033[0m')
                        while rosbag_reading.pm.is_alive():
                            # print('is still alive...')
                            pass
                        self.process_detect_label.configure(text='Completed Successfully!',
                                                            state='normal',
                                                            text_color='green',
                                                            font=customtkinter.CTkFont(
                                                                size=12, weight='bold')
                                                            )
                        self.process_show_results_button.configure(state =  'normal')
                        print('finished the reading bagfile process')
                        # print('\033[93mRosbag reading finished.. Now stopping marker detection..\033[0m')
                        rostopic_process.terminate()
                        marker_detection.shutdown()

                        # Remove the processes from running_processes dictionary
                        self.running_processes.pop(
                            f"{camera_name}_rosbag_reading")
                        self.running_processes.pop(
                            f"{camera_name}_marker_detection")
                    # self.running_processes.pop(f"{camera_name}_rostopic_process")
                        # try:
                        #     if self.csv_plotting_v2(csv_file_path) is not None:
                        #         print(
                        #         '\033[93mRosbag reading finished.. Marker detection and rostopic process stopped..\033[0m')
                        # except:
                        #         return

                        print('\033[93mRosbag reading finished..')
                        print('Marker detection and rostopic process stopped..\033[0m')

                    except roslaunch.RLException as e_error:
                        rospy.logerr(
                            f"Error:{e_error} in running :{camera_name}_marker_detection")
                except roslaunch.RLException as e_error:
                    rospy.logerr(
                        f"Error:{e_error} in reading and marker detection from:{camera_name}_rosbag_reading")
    def csv_plotting_v2(self, csv_file):
        fs = 200
        camera, path = self.get_camera_name(csv_file)
        data = pd.read_csv(csv_file)
        jpg_file = csv_file.replace('.csv', '.png')
        try:
            camera_name = data['field.header.frame_id']
            marker_id = data['field.transforms0.fiducial_id']
            image_seq = data['field.image_seq']
            x_disp = data['field.transforms0.transform.translation.x']
            y_disp = data['field.transforms0.transform.translation.y']
            z_disp = data['field.transforms0.transform.translation.z']
            
            x_disp = data['field.transforms0.transform.translation.x'] - data['field.transforms0.transform.translation.x'][0]
            y_disp = data['field.transforms0.transform.translation.y'] - data['field.transforms0.transform.translation.y'][0]
            z_disp = data['field.transforms0.transform.translation.z'] - data['field.transforms0.transform.translation.z'][0]
            
            x_disp = x_disp - np.mean(x_disp)
            y_disp = y_disp - np.mean(y_disp)
            z_disp = z_disp - np.mean(z_disp)
            
            # Plotting
            fig, axs = plt.subplots(2, 1, figsize=(10, 8))
            
            # Time domain plot
            axs[0].plot(x_disp, label='x_disp')
            axs[0].plot(y_disp, label='y_disp')
            axs[0].plot(z_disp, label='z_disp')
            axs[0].set_xlabel('Time (s)')
            axs[0].set_ylabel('Displacement (m)')
            axs[0].set_title('Camera: {}, Marker ID: {}, Image Seq: {}'.format(camera_name[0], marker_id[0], image_seq[0]))
            axs[0].legend()
            
            # Frequency domain plot
            n = len(x_disp)
            yf = fft(x_disp)
            xf = fftfreq(n, 1/fs)[:n//2]
            axs[1].plot(xf, 2.0/n * np.abs(yf[0:n//2]), label='x_disp')
            yf = fft(y_disp)
            axs[1].plot(xf, 2.0/n * np.abs(yf[0:n//2]), label='y_disp')
            yf = fft(z_disp)
            axs[1].plot(xf, 2.0/n * np.abs(yf[0:n//2]), label='z_disp')
            axs[1].set_xlabel('Frequency (Hz)')
            axs[1].set_ylabel('Magnitude')
            axs[1].set_title('Frequency domain')
            axs[1].legend()
            
            # Save the figure
            plt.savefig(jpg_file)
            
        except Exception as e:
            print('Error: ', str(e))
        
        
    def csv_plotting(self, csv_file):
        """This function plots the data from the csv file passed as input argument"""
        fs = 200
        upsample_factor = 200
        camera, path = self.get_camera_name(csv_file)
        data = pd.read_csv(csv_file)
        jpg_file = csv_file.replace('.csv', '.png')
        try:    
            # Extract the columns from the csv file
            camera_name = data['field.header.frame_id']
            marker_id = data['field.transforms0.fiducial_id']
            image_seq = data['field.image_seq']
            x_disp = data['field.transforms0.transform.translation.x'] 
            y_disp = data['field.transforms0.transform.translation.y'] 
            z_disp = data['field.transforms0.transform.translation.z']
            x_disp = data['field.transforms0.transform.translation.x'] - data['field.transforms0.transform.translation.x'][0]
            y_disp = data['field.transforms0.transform.translation.y'] - data['field.transforms0.transform.translation.y'][0]
            z_disp = data['field.transforms0.transform.translation.z'] - data['field.transforms0.transform.translation.z'][0]
            
            x_disp =  x_disp - np.mean(x_disp)
            y_disp = y_disp - np.mean(y_disp)
            z_disp = z_disp - np.mean(z_disp)
            

            
            
            # finding the length of x_disp
            # len_disp = len(x_disp)
            
            time_vector = range(len(x_disp))

            # upsample the time vector
            up_factor = upsample_factor
            # upsampled_time = np.linspace(0, len(t) - 1, len(t) * up_factor)
            upsampled_time = np.linspace(
                time_vector[0], time_vector[-1], len(time_vector) * up_factor)

            # # Now, upsample the displacement signals using linear interpolation.
            # x_disp_up = np.interp(upsampled_time, time_vector, x_disp)
            # y_disp_up = np.interp(upsampled_time, time_vector, y_disp)
            # z_disp_up = np.interp(upsampled_time, time_vector, z_disp)

            x_disp_up = x_disp
            y_disp_up = y_disp
            z_disp_up = z_disp

            f_x_disp, Pxx = signal.csd(x_disp_up, x_disp_up, fs, nfft=2048)
            f_y_disp, Pyy = signal.csd(y_disp_up, y_disp_up, fs, nfft=2048)
            f_z_disp, Pzz = signal.csd(z_disp_up, z_disp_up, fs, nfft=2048)

            fig, axis_plot = plt.subplots(nrows=3, ncols=2)

            # set the title of the figure
            fig.suptitle(
                f'3D displacement using ARUCO marker detection - {camera}')

            # Plotting the x, y, z displacement
            axis_plot[0, 0].plot([t/100 for t in time_vector], x_disp, 'r')
            axis_plot[0, 0].set_title('X displacement', loc='left', fontdict={
                'fontsize': 12, 'fontweight': 'bold'})
            axis_plot[0, 0].set_xlabel('Time (sec)')
            
            

            axis_plot[1, 0].plot([t/100 for t in time_vector], y_disp, 'g')
            axis_plot[1, 0].set_title('Y displacement', loc='left', fontdict={
                'fontsize': 12, 'fontweight': 'bold'})
            axis_plot[1, 0].set_xlabel('Time (sec)')

            axis_plot[2, 0].plot([t/100 for t in time_vector], z_disp, 'b')
            axis_plot[2, 0].set_title('Z displacement', loc='left', fontdict={
                'fontsize': 12, 'fontweight': 'bold'})
            axis_plot[2, 0].set_xlabel('Time (sec)')

            axis_plot[0, 0].grid(True)
            axis_plot[1, 0].grid(True)
            axis_plot[2, 0].grid(True)

            # Plotting the frequency response
            axis_plot[0, 1].semilogy(f_x_disp, np.abs(Pxx))
            axis_plot[0, 1].set_title('X-disp freq response', loc='left', fontdict={
                'fontsize': 12, 'fontweight': 'bold'})
            axis_plot[0, 1].set_xlabel('Frequency (Hz)')

            axis_plot[1, 1].semilogy(f_y_disp, np.abs(Pyy))
            axis_plot[1, 1].set_title('Y-disp freq response', loc='left', fontdict={
                'fontsize': 12, 'fontweight': 'bold'})
            axis_plot[1, 1].set_xlabel('Frequency (Hz)')

            axis_plot[2, 1].semilogy(f_z_disp, np.abs(Pzz))
            axis_plot[2, 1].set_title('Z-disp freq response', loc='left', fontdict={
                'fontsize': 12, 'fontweight': 'bold'})
            axis_plot[2, 1].set_xlabel('Frequency (Hz)')

            axis_plot[0, 1].grid(True)
            axis_plot[1, 1].grid(True)
            axis_plot[2, 1].grid(True)

            # Saving the plot with a given name
            fig.savefig(jpg_file)
            print('Close the figure to continue..')
            plt.show()
        except Exception as e:
            rospy.logerr('Error in reading file/detection: %s', e)
            return False

    def load_last_saved_button_event(self):
        """This function is called when the detect button is clicked."""
        print(
            '\033[92m*****************************************************************')
        print('********  Loading last saved bag file for post-processing  ******')
        print('*****************************************************************')

        try:
            if self.last_recorded_bag_file_name_with_path != "":
                # filename = os.fspath(self.last_recorded_bag_file_name)
                # print(
                #     f'Processing File: "{os.path.basename(filename)}" from directory: "{self.bagfile_path}"')
                print("\033[93m")
                print(f"Camera: {self.camera_selection_var}")
                print(f"Marker Size: {self.var_marker_size}")
                print(f"Recorded Time: {self.recorded_datetime_var}")
                print(
                    f"Loaded Bag File: {self.last_recorded_bag_file_name_with_path}")

                print(
                    f"\033[92m\nSuccessfully loaded bag file recorded from {self.camera_selection_var.get()}!")
                print(
                    '*****************************************************************\n\033[0m')
                self.loaded_last_file_flag = True
                # print(self.single_camera_dur)

                # if self.opened_bagfile_var == self.last_recorded_bag_file_name_path:
                #     print("Loaded file is the last recorded file")
                # else:
                #     print("Loaded file is not the last recorded file")
            else:
                raise NameError(
                    "Variable self.last_recorded_bag_file_name_with_path is not defined or is an empty string.")
        except NameError:
            rospy.logerr(
                f"No file {self.read_bag_launch} selected, please record a bag file first.")

    def camera_calibration(self, cam_number):
        cb_dim = self.board_size
        sq_size = self.square_size
        print('****************************************************')
        print('********  Starting Camera 1 Calibration     ********')
        print('****************************************************')
        print('Board Size: ', cb_dim)
        print('Square Size: ', sq_size)
        print("Starting Camera Calibration for Camera ", cam_number, '...\n')
        cam_name = {1: 'camera_1',
                    2: 'camera_2',
                    3: 'camera_3'}

        print(cam_name[cam_number])

        # print("Board Size: ", cb_dim, "manual: ", self.sidebar_entry_get_calib_cb_dim.get())
        # print("Square Size: ", sq_size, "manual: ", self.sidebar_entry_get_calib_sq_size.get())
        # return
        self.sidebar_camera_btn_event(cam_number, False, False)

        cmd = ['rosrun', 'camera_calibration', 'cameracalibrator.py',
               '--size', cb_dim, '--square', sq_size, '--k-coefficients=2',
               '--fix-principal-point', 'i', '--fix-aspect-ratio',
               f'image:=/{cam_name[cam_number]}/image_raw', f'camera:=/{cam_name[cam_number]}']

        # Execute the command
        subprocess.call(cmd)
        self.sidebar_camera_btn_event(cam_number, False, False)

    def exit_button_click(self):
        """This function is called when the exit button is clicked."""
        print("Terminated successfully.")
        # subprocess.call('pkill roscore', shell=True)
        # self.destroy()
        # os.system("xdotool key ctrl+shift+w")

        exit()
    def sidebar_set_dictionary_btn_event(self):
        """This function is called when the set dictionary button is clicked."""
        print("Setting detection parameters...")
        self.var_marker_size = self.sidebar_marker_size_entry.get()
        self.var_dictionary = self.sidebar_dictionary_entry.get()
        print("Dictionary set to: ", self.var_dictionary)
        print("Marker size set to: ", self.var_marker_size)
        
    def show_results_button_event(self):
        # path_image = '/home/agcam/ros_ws/src/gige_cam_driver/csvfiles/camera_1_20s_2023-04-05_11-34-18.png'
        # image = Image.open(path_image)
        # image = image.resize((800, 600), Image.ANTIALIAS)
        # photo = ImageTk.PhotoImage(image)
        # label = customtkinter.CTkLabel(master=self.tabview.tab("Display Results"), image=photo)
        # label.image = photo
        # label.pack()
        pass
    
        

if __name__ == "__main__":
    app = GUI()
    app.mainloop()
