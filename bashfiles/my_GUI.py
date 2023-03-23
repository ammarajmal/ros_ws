#!/usr/bin/env python3
import time
import tkinter as tk
from tkinter import filedialog
import os
import roslaunch
import rospkg
import rospy
import customtkinter
import subprocess
import datetime


subprocess.Popen(['gnome-terminal', '--', '/bin/bash', '-c', 'source /opt/ros/noetic/setup.bash && roscore; exec /bin/bash'])


themes = {'blue': ("#3B8ED0", "#1F6AA5"),
          'green': ("#2CC985", "#2FA572"),
          'dark-blue': ("#3a7ebf", "#1f538d")
}
# select value of color_select from (0: blue, 1: green, 2: dark-blue)
color_select = list(themes.keys())[0]

# Modes: "System" (standard), "Dark", "Light"
customtkinter.set_appearance_mode("System")
# Themes: "blue" (standard), "green", "dark-blue"
customtkinter.set_default_color_theme(color_select)


class GUI(customtkinter.CTk):
    """_summary_

    Args:
        customtkinter (_type_): _description_
    """

    def __init__(self) -> None:
        super().__init__()

        rospy.init_node('launch_handle', anonymous=False)
        self.package = 'gige_cam_driver'
        self.check_camera_1_var = tk.StringVar(self, "on")
        self.check_camera_2_var = tk.StringVar(self, "on")
        self.check_camera_3_var = tk.StringVar(self, "on")
        self.camera_selection_var = tk.StringVar(self, "Camera 1")
        self.single_camera_duration_var = tk.StringVar(self, 'Select')
        self.multi_camera_duration_var = tk.StringVar(self, 'Select')
        self.bagfile_var = tk.StringVar(self)
        
        self.sidebar_entry_get_calib_sq_size_var = tk.StringVar()
        self.sidebar_entry_get_calib_cb_dim_var = tk.StringVar()
        self.board_size = "6x5"
        self.square_size = "0.025"
        
        
        # ********************************************************************************
        # Path management for Launch files
        # ********************************************************************************

        self.launch_path = rospkg.RosPack().get_path('gige_cam_driver') + '/launch/'
        self.detect_launch_path = rospkg.RosPack().get_path('aruco_detect') + '/launch/'
        self.bagfile_path = self.launch_path.replace("launch/", "bagfiles/")

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

        self.record_bag_launch = f"{self.launch_path}recordbag.launch"
        self.read_bag_launch = f"{self.launch_path}readbag.launch"
        self.cam_launch = f"{self.launch_path}cam.launch"
        self.view_launch = f"{self.launch_path}viewcam.launch"
        self.calib_launch = f"{self.launch_path}calib.launch"
        self.detect_launch = f"{self.detect_launch_path}detect.launch"
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
        self.main_label.grid(row=0, column=1, padx=10, pady=(10,0), sticky="nsew")

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
        self.sidebar_frame_cam_calib.grid(row=1, column=0, rowspan=5, pady=10, padx=10)
        self.sidebar_frame_cam_view.grid(row=6, column=0, rowspan=3, pady=10, padx=10)
        self.sidebar_frame_ui.grid(row=9, column=0, rowspan=2, pady=10, padx=10)
        self.sidebar_frame_cam_calib_label = customtkinter.CTkLabel(
            master=self.sidebar_frame,
            text="Camera Calibration",
            font=customtkinter.CTkFont(size=20, weight="normal"),
            text_color='black'
        )
        self.sidebar_frame_cam_calib_label.grid(row=0, column=0, padx=30, pady=(20, 5), sticky="nsew")
        self.sidebar_btn_cam_1_calib = customtkinter.CTkButton(
            master=self.sidebar_frame_cam_calib,
            text='Calibrate Camera 1',
            command=lambda: self.sidebar_camera_btn_event(1, False, True)
        )
        self.sidebar_btn_cam_2_calib = customtkinter.CTkButton(
            master=self.sidebar_frame_cam_calib,
            text='Calibrate Camera 2',
            command=lambda: self.sidebar_camera_btn_event(2, False, True)
        )
        self.sidebar_btn_cam_3_calib = customtkinter.CTkButton(
            self.sidebar_frame_cam_calib,
            text='Calibrate Camera 3',
            command=lambda: self.sidebar_camera_btn_event(3, False, True)
        )
        self.sidebar_entry_get_calib_cb_dim_label = customtkinter.CTkLabel(
            master=self.sidebar_frame_cam_calib,
            text="Board size (wxh):"
        )
        self.sidebar_entry_get_calib_cb_dim = customtkinter.CTkEntry(
            master= self.sidebar_frame_cam_calib,
            placeholder_text="6x5",
            placeholder_text_color="#808080",
            width=50
        )
        self.sidebar_entry_get_calib_sq_size_label = customtkinter.CTkLabel(
            master=self.sidebar_frame_cam_calib,
            text="Square size (m):"
        )
        self.sidebar_entry_get_calib_sq_size = customtkinter.CTkEntry(
            master= self.sidebar_frame_cam_calib,
            placeholder_text="0.025",
            placeholder_text_color="#808080",
            width=50
        )
        self.sidebar_btn_set_calib_success_label = customtkinter.CTkLabel(
            master=self.sidebar_frame_cam_calib,
            text="☑",
            text_color='green',
            font=customtkinter.CTkFont(size=25, weight="bold"),
            
        )
        self.sidebar_btn_set_calib = customtkinter.CTkButton(
            self.sidebar_frame_cam_calib,
            text='Update',
            command=self.sidebar_btn_set_calib_event,
            width=85
            
        )
        self.sidebar_entry_get_calib_cb_dim_label.grid  (row=1, column=0, padx=(20, 0), pady=(10, 5), sticky="nsw")
        self.sidebar_entry_get_calib_sq_size_label.grid (row=2, column=0, padx=(20, 0), pady=0, sticky="nsw")
        self.sidebar_entry_get_calib_cb_dim.grid        (row=1, column=1, padx=(10, 10), pady=(10,5), sticky="nsw")
        self.sidebar_entry_get_calib_sq_size.grid       (row=2, column=1, padx=(10, 10), pady=0, sticky="nsw")
        self.sidebar_btn_set_calib.grid                 (row=3, column=0,padx=(10, 10), pady=(10,10), sticky='e')
        self.sidebar_btn_set_calib_success_label.grid   (row=3, column=1,padx=(20, 0), pady=(10,10), sticky='nsw')
        self.sidebar_btn_cam_1_calib.grid               (row=4, column=0, columnspan=2,padx=10, pady=10)
        self.sidebar_btn_cam_2_calib.grid               (row=5, column=0, columnspan=2,padx=10, pady=0)
        self.sidebar_btn_cam_3_calib.grid               (row=6, column=0, columnspan=2,padx=10, pady=(10,20))
        self.sidebar_frame_cam_view_label = customtkinter.CTkLabel(
            master=self.sidebar_frame_cam_view,
            text="Camera View"
        )
        self.sidebar_btn_cam_1_start = customtkinter.CTkButton(
            master=self.sidebar_frame_cam_view,
            text='Start Camera 1',
            command=lambda: self.sidebar_camera_btn_event(1, False, False)
        )
        self.sidebar_btn_cam_2_start = customtkinter.CTkButton(
            master=self.sidebar_frame_cam_view,
            text='Start Camera 2',
            command=lambda: self.sidebar_camera_btn_event(2, True, False)
        )
        self.sidebar_btn_cam_3_start = customtkinter.CTkButton(
            master=self.sidebar_frame_cam_view,
            text='Start Camera 3',
            command=lambda: self.sidebar_camera_btn_event(3,True, False)
        )
        self.sidebar_frame_cam_view_label.grid  (row=0, column=0, columnspan=2, padx=25, pady=(10, 10))
        self.sidebar_btn_cam_1_start.grid       (row=1, column=0, columnspan=2, padx=25, pady=(0,10))
        self.sidebar_btn_cam_2_start.grid       (row=2, column=0, columnspan=2, padx=25, pady=0)
        self.sidebar_btn_cam_3_start.grid       (row=3, column=0, columnspan=2, padx=25, pady=(10, 20))
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
        self.ui_color_optionmenu = customtkinter.CTkOptionMenu(
            master=self.sidebar_frame_ui,
            values=["Blue", "Dark Blue", "Green"],
            command=self.change_color_event,
            width=40
        )
        self.ui_color_optionmenu_label = customtkinter.CTkLabel(
            master=self.sidebar_frame_ui,
            text="UI Color:"
        )
        self.appearance_mode_label.grid         (row=0, column=0, padx=(16, 0), pady=(10, 5), sticky="nsw")
        self.appearance_mode_optionemenu.grid   (row=0, column=1, padx=(0,10), pady=(10, 5), sticky="nse")
        self.ui_color_optionmenu_label.grid     (row=1, column=0, padx=(16, 0), pady=(10, 5), sticky="nsw")
        self.ui_color_optionmenu.grid           (row=1, column=1, padx=(0,10), pady=(10, 5), sticky="nse")

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
        self.tabview.grid           (row=1, column=1, padx=20, pady=(5,10), sticky="nsew")
        self.main_button_exit.grid  (row=2, column=1, padx=(20, 20), pady=(0, 20))

        self.tabview.add("Record & Process Data")
        self.tabview.add("Display Results")
        self.tabview.tab("Record & Process Data").grid_columnconfigure(0, weight=1)  # configure grid of individual tabs
        

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
        self.record_label.grid              (row=0, column=0, padx=20, pady=(5, 0), sticky="nsew")
        self.record_single_frame.grid       (row=1, column=0, padx=20, pady=(10,0), sticky="nsew")
        self.record_multiple_frame.grid     (row=2, column=0, padx=20, pady=(20,0), sticky="nsew")
        self.process_label.grid             (row=3, column=0, padx=20, pady=10,     sticky="nsew")
        self.process_frame.grid             (row=4, column=0, padx=20, pady=(0,20), sticky="nsew")
        # self.record_multiple_frame.rowconfigure(0, weight=1)

        self.process_load_options_label = customtkinter.CTkLabel(
            master=self.process_frame,
            text="Load Data:",
            font=customtkinter.CTkFont(size=14)
        )
        self.process_load_button = customtkinter.CTkButton(
            master=self.process_frame,
            text="Load File",
            font=customtkinter.CTkFont(size=14),
            command=self.load_data_button_event
        )
        self.process_detect_button = customtkinter.CTkButton(
            master=self.process_frame,
            text="Post-Process Data",
            font=customtkinter.CTkFont(size=14),
            command=self.detect_button_event
        )
        
        self.process_load_options_label.grid(row=0, column=0, padx=10, pady=(5,5),   sticky="nsew")
        self.process_load_button.grid       (row=0, column=1, padx=10, pady=(5,5),   sticky="nsew")
        self.process_detect_button.grid     (row=0, column=2, padx=10, pady=(5,5),   sticky="nsew")

         
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
            justify = 'center',
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



        self.single_cam_label.grid                  (row=0, column=1, padx=10, pady=(10,10), sticky="nsew", columnspan=2, rowspan=1)
        self.single_cam_select_label.grid           (row=1, column=0, padx=10, pady=(5,5),   sticky="nsew")
        self.single_camera_1_radio.grid             (row=1, column=1, padx=5,  pady=(0,5),   sticky="nsew")
        self.single_camera_2_radio.grid             (row=1, column=2, padx=5,  pady=(0,5),   sticky="nsew")
        self.single_camera_3_radio.grid             (row=1, column=3, padx=5,  pady=(0,5),   sticky="nsew")
        self.single_dur_select_label.grid           (row=2, column=0, padx=(20, 5),  pady=(0,5),   sticky="nsew")
        self.single_camera_dur_combo_box.grid       (row=2, column=1, padx=0,  pady=(0,5),   sticky="nsew")
        self.single_dur_select_or_label.grid        (row=2, column=2, padx=(0, 10),  pady=(0,5),   sticky="nse")
        self.single_camera_dur_entry.grid           (row=2, column=3, padx=(0, 20),  pady=(0,5),   sticky="nsew")
        self.single_camera_rec_button.grid          (row=3, column=1, padx=(10), pady=(10, 20),sticky="nsew")
        self.single_rec_manual_label.grid           (row=3, column=2, padx=(0, 10), pady=(10, 20),sticky="nsew")
        self.single_camera_rec_manual_button.grid   (row=3, column=3, padx=(0, 20),  pady=(10, 20),sticky="nsew")
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
            command=self.checkbox_event,
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
            command=self.checkbox_event,
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
            command=self.checkbox_event,
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
        self.multi_camera_rec_button = customtkinter.CTkButton(
            master=self.record_multiple_frame,
            text="Record",
            font=customtkinter.CTkFont(size=14)
        )
        self.multi_camera_rec_manual_button = customtkinter.CTkButton(
            master=self.record_multiple_frame,
            text="Record",
            font=customtkinter.CTkFont(size=14),
            fg_color=("#DCE4EE", "gray10"),
            border_width=2,
            text_color=("gray10", "#DCE4EE")
        )



        self.multiple_cams_label.grid               (row=0, column=1, padx=10, pady=(10,10), sticky="nsew", columnspan=2, rowspan=1)
        self.multi_cam_select_label.grid            (row=1, column=0, padx=10, pady=(5,5),   sticky="nsew")
        self.multi_camera_1_checkbox.grid           (row=1, column=1, padx=5,  pady=(0,5),   sticky="nsew")
        self.multi_camera_2_checkbox.grid           (row=1, column=2, padx=5,  pady=(0,5),   sticky="nsew")
        self.multi_camera_3_checkbox.grid           (row=1, column=3, padx=5,  pady=(0,5),   sticky="nsew")
        self.multi_dur_select_label.grid            (row=2, column=0, padx=(20,5),  pady=(0,5),   sticky="nsew")
        self.multi_camera_dur_combo_box.grid        (row=2, column=1, padx=0,  pady=(0,5),   sticky="nsew")
        self.multi_dur_select_or_label.grid         (row=2, column=2, padx=(0, 10),  pady=(0,5),   sticky="nsew")
        self.multi_camera_dur_entry.grid            (row=2, column=3, padx=(0, 20),  pady=(0,5),   sticky="nsew")
        self.multi_camera_rec_button.grid           (row=3, column=1, padx=10, pady=(10, 20),  sticky="nsew")
        self.multi_rec_manual_label.grid            (row=3, column=2, padx=(0, 10), pady=(10, 20),sticky="nsew")
        self.multi_camera_rec_manual_button.grid    (row=3, column=3, padx=(0, 20),  pady=(10, 20),sticky="nsew")





    def checkbox_event(self):
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
        calib_launch_args = [self.calib_launch,
                            f"cam:={camera_name}",
                            f"size:={self.board_size}",
                            f"square:={self.square_size}"
        ]

        # Create a ROS launch file with the camera launch command
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(camera_launch_args)[0], camera_launch_args[1:])]
        cam_driver = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)

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
                view_launch_file = [(roslaunch.rlutil.resolve_launch_arguments(view_launch_args)[0], view_launch_args[1:])]
                view_output = roslaunch.parent.ROSLaunchParent(self.uuid, view_launch_file)
                view_output.start()
                self.running_processes[f'{camera_name}_view'] = view_output

            # If calibrate_camera is True, start camera calibration
            if calibrate_camera:
                rospy.loginfo(f"{camera_name} calibration started successfully.")
                calibrate_launch_file = [(roslaunch.rlutil.resolve_launch_arguments(calib_launch_args)[0], calib_launch_args[1:])]
                camera_calibrate = roslaunch.parent.ROSLaunchParent(self.uuid, calibrate_launch_file)
                camera_calibrate.start()
                self.running_processes[f'{camera_name}_calibrate'] = camera_calibrate

        except roslaunch.RLException as excep_camera:
            rospy.logerr(f"Error starting {camera_name} camera driver: {str(excep_camera)}")
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
            if f'{camera_name}_calibrate' in self.running_processes:
                self.running_processes[f'{camera_name}_calibrate'].shutdown()
        except roslaunch.RLException as excep_camera:
            rospy.logerr(f"Error stopping {camera_name} camera driver: {str(excep_camera)}")
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
        self.running_processes.pop(f'{camera_name}_calibrate', None)
        # Print success message
        
        rospy.loginfo(f"{camera_name} camera driver stopped successfully.")


    def sidebar_camera_btn_event(self, camera_number, show_camera, calibrate_camera):
        """This function is called when a camera button is pressed"""
        cameras = [
            {'camera_name': 'camera_1', 'device_id': 0, 'calibration_file': 'cam1', 'button': self.sidebar_btn_cam_1_start, 'calibrate_button': self.sidebar_btn_cam_1_calib, 'name': 'Camera 1'},
            {'camera_name': 'camera_2', 'device_id': 1, 'calibration_file': 'cam2', 'button': self.sidebar_btn_cam_2_start, 'calibrate_button': self.sidebar_btn_cam_2_calib, 'name': 'Camera 2'},
            {'camera_name': 'camera_3', 'device_id': 2, 'calibration_file': 'cam3', 'button': self.sidebar_btn_cam_3_start, 'calibrate_button': self.sidebar_btn_cam_3_calib, 'name': 'Camera 3'}
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
        camera_active_states = [self.camera_1_active, self.camera_2_active, self.camera_3_active]
        camera_active = camera_active_states[camera_number - 1]

        # Start or stop the camera depending on its current state
        try:
            if not camera_active:
                # Start the selected camera
                self.start_camera(camera_name, device_id, calibration_file, show_camera, calibrate_camera)


                # Update button text and color
                button.configure(text=f"Stop {camera['name']}", fg_color=("#fa5f5a", "#ba3732"))

                # Set the camera active flag to True
                camera_active_states[camera_number - 1] = True
            else:
                # Stop the selected camera
                self.stop_camera(camera_name)

                # Update button text and color
                button.configure(text=f"{button_name} {camera['name']}", fg_color=themes[color_select])

                # Set the camera active flag to False
                camera_active_states[camera_number - 1] = False
        except Exception as excep_camera:
            rospy.logerr(f"Error {'' if camera_active else 'starting'} {camera_name} camera: {str(excep_camera)}")
            return

        # Update the camera active states
        self.camera_1_active, self.camera_2_active, self.camera_3_active = camera_active_states

    def sidebar_btn_set_calib_event(self):
        """This function is called when the set calibration button is pressed and updates the calibration values"""
        self.board_size = self.sidebar_entry_get_calib_cb_dim.get()
        self.square_size = self.sidebar_entry_get_calib_sq_size.get()

    def start_camera_record(self, camera_name, device_id, calibration_file, dur):
        """Starts a camera driver and optionally a camera view"""
        time_dur_bag = dur
        camera_num = device_id + 1
        datetime_var = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        # bagfile_name = f"cam{camera_num}_bagfile"

        camera_launch_args = [f"{self.cam_launch}",
                            f"cam:={camera_name}",
                            f"device_id:={device_id}",
                            f"calib_file:={calibration_file}"
        ]
        record_launch_args = [
                                self.record_bag_launch,
                                f'cam:=camera_{camera_num}',
                                f'dur:={time_dur_bag}',
                                f'bagfile_datetime:={datetime_var}'
                            ]
        # Create a ROS launch file with the camera launch command
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(camera_launch_args)[0], camera_launch_args[1:])]
        cam_driver = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)

        # Create a ROS launch file with the camera record launch command
        record_single_cam_file = [(roslaunch.rlutil.resolve_launch_arguments(record_launch_args)[0], record_launch_args[1:])]
        record_single_cam = roslaunch.parent.ROSLaunchParent(self.uuid, record_single_cam_file)


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
            record_single_cam.start()
            self.running_processes[f'{camera_name}_record'] = record_single_cam
            
            while record_single_cam.pm.is_alive():
                rospy.sleep(1)
                print(f"recording {camera_name}...")
            

            # Shutdown the camera driver
            try:
                self.running_processes[f'{camera_name}_driver'].shutdown()
                if f'{camera_name}_record' in self.running_processes:
                    self.running_processes[f'{camera_name}_record'].shutdown()

            except roslaunch.RLException as excep_camera:
                rospy.logerr(f"Error stopping {camera_name} camera driver: {str(excep_camera)}")
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
            
            


        except roslaunch.RLException as excep_camera:
            rospy.logerr(f"Error starting {camera_name} camera driver: {str(excep_camera)}")
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
            rospy.logerr(f"Error stopping {camera_name} camera driver: {str(excep_camera)}")
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
        
        cameras = [
            {'camera_name': 'camera_1', 'device_id': 0, 'calibration_file': 'cam1', 'button': self.sidebar_btn_cam_1_start},
            {'camera_name': 'camera_2', 'device_id': 1, 'calibration_file': 'cam2', 'button': self.sidebar_btn_cam_2_start},
            {'camera_name': 'camera_3', 'device_id': 2, 'calibration_file': 'cam3', 'button': self.sidebar_btn_cam_3_start}
        ]
        camera_selected = self.camera_selection_var.get()
        if camera_selected == "Camera 1":
            camera_number = 1
        elif camera_selected == "Camera 2":
            camera_number = 2
        else:
            camera_number = 3
            
        man_dur = self.single_camera_dur_entry.get()
        combo_dur = self.single_camera_dur_combo_box.get()
        single_camera_dur = ''
        print("Camera: ", camera_selected)
        
        if man_dur == "":
            if combo_dur == "Select":
                print("Please select a duration")
            else:
                single_camera_dur = combo_dur
                print(f"Duration: {single_camera_dur}")
        else:
            single_camera_dur = man_dur
            print(f"Duration: {single_camera_dur}")
            
            
        

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
        camera_active_states = [self.camera_1_active, self.camera_2_active, self.camera_3_active]
        camera_active = camera_active_states[camera_number - 1]

        # Start or stop the camera depending on its current state
        try:
            if not camera_active:

                # Update button color
                button.configure(fg_color=("#fa5f5a", "#ba3732"))

                # Set the camera active flag to True
                camera_active_states[camera_number - 1] = True

                # Start the selected camera
                self.start_camera_record(camera_name, device_id, calibration_file, single_camera_dur)


                # Update button color
                button.configure(fg_color=themes[color_select])

                # Set the camera active flag to False
                camera_active_states[camera_number - 1] = False

        except Exception as excep_camera:
            rospy.logerr(f"Error {'' if camera_active else 'starting'} {camera_name} camera: {str(excep_camera)}")
            return

        # Update the camera active states
        self.camera_1_active, self.camera_2_active, self.camera_3_active = camera_active_states

    def load_data_button_event(self):
        """This function is called when the load data button is clicked."""
        print("Loading data...")
        root = tk.Tk()
        root.withdraw()
        self.bagfile_var = filedialog.askopenfilename(initialdir=self.bagfile_path)
        if self.bagfile_var != "":
            print(f'Loadded File: "{os.path.basename(self.bagfile_var)}" from directory: "{self.bagfile_path}"' )

            # self.load_data(filepath)
        
    def detect_button_event(self):
        """This function is called when the detect button is clicked."""
        print("Post-processing started...")
        try:
            if self.bagfile_var != "":
                filename = os.fspath(self.bagfile_var)
                print(f'Processing File: "{os.path.basename(filename)}" from directory: "{self.bagfile_path}"')
        except TypeError:
            print("Error: No file selected, Please load a bag file first.")

                    
        
        
    def exit_button_click(self):
        """This function is called when the exit button is clicked."""
        print("Terminated successfully.")
        subprocess.call('pkill roscore', shell=True)
        self.destroy()
        os.system("xdotool key ctrl+shift+w")

        exit()


if __name__ == "__main__":
    app = GUI()
    app.mainloop()
