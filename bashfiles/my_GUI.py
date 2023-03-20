#!/usr/bin/env python3
import time
import tkinter as tk
import roslaunch
import rospkg
import rospy
import customtkinter

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
        self.camera_selection_var = tk.StringVar()
        self.sidebar_entry_get_calib_sq_size_var = tk.StringVar()
        self.sidebar_entry_get_calib_cb_dim_var = tk.StringVar()
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
            command=lambda: self.sidebar_button_event(1),
        )
        self.sidebar_btn_cam_2_calib = customtkinter.CTkButton(
            master=self.sidebar_frame_cam_calib,
            text='Calibrate Camera 2',
            command=lambda: self.sidebar_button_event(2)
        )
        self.sidebar_btn_cam_3_calib = customtkinter.CTkButton(
            self.sidebar_frame_cam_calib,
            text='Calibrate Camera 3',
            command=lambda: self.sidebar_button_event(3)
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
            font=customtkinter.CTkFont(size=25, weight="bold")
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
            command=self.sidebar_btn_cam_1_start_event
        )
        self.sidebar_btn_cam_2_start = customtkinter.CTkButton(
            master=self.sidebar_frame_cam_view,
            text='Start Camera 2',
            command=self.sidebar_btn_cam_2_start_event
        )
        self.sidebar_btn_cam_3_start = customtkinter.CTkButton(
            master=self.sidebar_frame_cam_view,
            text='Start Camera 3',
            command=self.sidebar_btn_cam_3_start_event
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

        # self.ui_color_optionmenu= customtkinter.CTkOptionMenu(
        #     master=self.sidebar_frame_ui,
        #     values=["Dark Blue", "Blue", "Green"],
        #     command=self.change_color_event,
        #     width=40
        # )
        self.appearance_mode_label.grid         (row=0, column=0, padx=(16, 0), pady=(10, 5), sticky="nsw")
        self.appearance_mode_optionemenu.grid   (row=0, column=1, padx=(0,10), pady=(10, 5), sticky="nse")
        self.ui_color_optionmenu_label.grid     (row=1, column=0, padx=(16, 0), pady=(10, 5), sticky="nsw")
        self.ui_color_optionmenu.grid           (row=1, column=1, padx=(0,10), pady=(10, 5), sticky="nse")


        # self.main_label.place(relx=0.6, rely=0.055, anchor='center')

        self.tabview = customtkinter.CTkTabview(
            master=self,
            fg_color='#C4C3C3'
        )

        self.tabview.grid(row=1, column=1, padx=20, pady=(5,10), sticky="nsew")
        
        self.main_button_exit = customtkinter.CTkButton(
            master=self,
            text='EXIT',
            command=self.exit_button_click,
            fg_color=("gray10", "#DCE4EE"),
            border_width=2,
            text_color=("#DCE4EE", "gray10")
        )
        self.main_button_exit.grid(row=2, column=1, padx=(20, 20), pady=(0, 20))


        self.tabview.add("Recording Data")
        self.tabview.add("Post-Processing")
        self.tabview.add("Displaying Results")
        self.tabview.tab("Recording Data").grid_columnconfigure(0, weight=1)  # configure grid of individual tabs
        self.tabview.tab("Post-Processing").grid_columnconfigure(0, weight=1)


        self.record_label = customtkinter.CTkLabel(
            master=self.tabview.tab("Recording Data"),
            text="Recording from Camera",
            font=customtkinter.CTkFont(size=16),
            text_color='black'
        )
        self.record_label.grid(row=0, column=0, columnspan=15, padx=20, pady=(5, 0), sticky="nsew")

        self.post_processing_label = customtkinter.CTkLabel(
            master= self.tabview.tab("Post-Processing"),
            text="Post-Processing",
            font=customtkinter.CTkFont(size=16),
        )
        self.post_processing_label.grid(row=0, column=0, columnspan=15, padx=20, pady=(5, 0), sticky="nsew")
        
        
        
        
        
        self.record_single_frame = customtkinter.CTkFrame(
            master=self.tabview.tab("Recording Data"),
            fg_color=('lightgray', 'gray')
        )
        self.record_multiple_frame = customtkinter.CTkFrame(
            master=self.tabview.tab("Recording Data"),
            fg_color=('lightgray', 'gray')
        )
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
        self.single_dur_select_label = customtkinter.CTkLabel(
            master=self.record_single_frame,
            text="Select duration:",
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
            text="or",
            font=customtkinter.CTkFont(size=14)
            # text_color="#707070"
        )




        self.single_camera_1_checkbox = customtkinter.CTkCheckBox(
            master=self.record_single_frame,
            text="Camera 1",
            font=customtkinter.CTkFont(size=14),
            checkbox_width=20,
            checkbox_height=20,
            command=self.checkbox_event,
            variable=self.camera_selection_var,
            onvalue="on",
            offvalue="off",
            # text_color="#808080"

        )
        self.single_camera_1_radio = customtkinter.CTkRadioButton(
                    master=self.record_single_frame,
            value="Camera 1",
            text="Camera 1",
            font=customtkinter.CTkFont(size=14),
            command=self.checkbox_event,
            variable=self.camera_selection_var,
            # text_color="#808080"
            )
        self.single_camera_2_radio = customtkinter.CTkRadioButton(
                    master=self.record_single_frame,
            value="Camera 2",
            text="Camera 2",
            font=customtkinter.CTkFont(size=14),
            command=self.checkbox_event,
            variable=self.camera_selection_var,
            # text_color="#808080"
            )
        self.single_camera_3_radio = customtkinter.CTkRadioButton(
                    master=self.record_single_frame,
            value="Camera 3",
            text="Camera 3",
            font=customtkinter.CTkFont(size=14),
            command=self.checkbox_event,
            variable=self.camera_selection_var,
            # text_color="#808080"
            )
        self.single_camera_dur_combo_box = customtkinter.CTkComboBox(
            master=self.record_single_frame,
            values=["Duration: 10s",
                    "Duration: 20s",
                    "Duration: 30s",
                    "Duration: 40s",
                    "Duration: 50s",
                    "Duration: 60s"
                    ],
            # font=customtkinter.CTkFont(size=14),
            # dropdown_text_color="#808080",
            # dropdown_fg_color='#ffffff',
            # text_color="#303030"
            )
        self.single_camera_dur_entry = customtkinter.CTkEntry(
            master=self.record_single_frame,
            placeholder_text="Manual entry",
            # placeholder_text_color="#808080",
            corner_radius=5,
            width=20
        )
        self.single_camera_rec_button = customtkinter.CTkButton(
            master=self.record_single_frame,
            text="Record",
            font=customtkinter.CTkFont(size=14)
        )
        self.single_camera_rec_manual_button = customtkinter.CTkButton(
            master=self.record_single_frame,
            text="Record",
            font=customtkinter.CTkFont(size=14),
            fg_color=("#DCE4EE", "gray10"),
            border_width=2,
            text_color=("gray10", "#DCE4EE")
        )



        self.single_cam_label.grid               (row=0, column=1, padx=10, pady=(10,10), sticky="nsew", columnspan=2, rowspan=1)
        self.single_cam_select_label.grid            (row=1, column=0, padx=10, pady=(5,5),   sticky="nsew")
        self.single_camera_1_radio.grid           (row=1, column=1, padx=5,  pady=(0,5),   sticky="nsew")
        self.single_camera_2_radio.grid           (row=1, column=2, padx=5,  pady=(0,5),   sticky="nsew")
        self.single_camera_3_radio.grid           (row=1, column=3, padx=5,  pady=(0,5),   sticky="nsew")
        self.single_dur_select_label.grid            (row=2, column=0, padx=5,  pady=(0,5),   sticky="nsew")
        self.single_camera_dur_combo_box.grid        (row=2, column=1, padx=5,  pady=(0,5),   sticky="nsew", columnspan=1)
        self.single_dur_select_or_label.grid         (row=2, column=2, padx=5,  pady=(0,5),   sticky="nsew")
        self.single_camera_dur_entry.grid            (row=2, column=3, padx=5,  pady=(0,5),   sticky="nsew")
        self.single_camera_rec_button.grid           (row=3, column=1, padx=10, pady=(5, 5),  sticky="nsew", columnspan=2)
        self.single_rec_manual_label.grid            (row=4, column=2, padx=10, pady=(10, 20),sticky="nsew")
        self.single_camera_rec_manual_button.grid    (row=4, column=3, padx=5,  pady=(10, 20),sticky="nsew")
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
            text="Select duration:",
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
            text="or",
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
            values=["Duration: 10s",
                    "Duration: 20s",
                    "Duration: 30s",
                    "Duration: 40s",
                    "Duration: 50s",
                    "Duration: 60s"
                    ],
            # font=customtkinter.CTkFont(size=14),
            # dropdown_text_color="#808080",
            # dropdown_fg_color='#ffffff',
            # text_color="#303030"
            )
        self.multi_camera_dur_entry = customtkinter.CTkEntry(
            master=self.record_multiple_frame,
            placeholder_text="Manual entry",
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
        self.multi_dur_select_label.grid            (row=2, column=0, padx=5,  pady=(0,5),   sticky="nsew")
        self.multi_camera_dur_combo_box.grid        (row=2, column=1, padx=5,  pady=(0,5),   sticky="nsew", columnspan=1)
        self.multi_dur_select_or_label.grid         (row=2, column=2, padx=5,  pady=(0,5),   sticky="nsew")
        self.multi_camera_dur_entry.grid            (row=2, column=3, padx=5,  pady=(0,5),   sticky="nsew")
        self.multi_camera_rec_button.grid           (row=3, column=1, padx=10, pady=(5, 5),  sticky="nsew", columnspan=2)
        self.multi_rec_manual_label.grid            (row=4, column=2, padx=10, pady=(10, 20),sticky="nsew")
        self.multi_camera_rec_manual_button.grid    (row=4, column=3, padx=5,  pady=(10, 20),sticky="nsew")




        self.record_single_frame.grid       (row=1, column=0, padx=20, pady=(10,20), sticky="nsew")
        self.record_multiple_frame.grid     (row=2, column=0, padx=20, pady=(10,20), sticky="nsew")
        # self.record_multiple_frame.rowconfigure(0, weight=1)


        self.tabview.tab("Recording Data").rowconfigure(1, weight=1)  # add this line to set row 1 to have equal weight
        self.tabview.tab("Recording Data").rowconfigure(2, weight=1)  # add this line to set row 2 to have equal weight
        self.tabview.tab("Recording Data").columnconfigure(0, weight=1)  # add this line to set column 0 to have equal weight

    def sidebar_btn_set_calib_event(self):
        pass
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

    def start_camera(self, camera_name, device_id, calibration_file, view_camera):
        """Starts a camera driver and optionally a camera view"""
        camera_launch_args = [f"{self.cam_launch}",
                              f"cam:={camera_name}",
                              f"device_id:={device_id}",
                              f"calib_file:={calibration_file}"
        ]
        view_launch_args = [self.view_launch,
                            f"camera_name:={camera_name}"
        ]

        # Create a ROS launch file with the camera launch command
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(camera_launch_args)[0], camera_launch_args[1:])]
        cam_driver = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)

        view_roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(view_launch_args)[0], view_launch_args[1:])]
        view_output = roslaunch.parent.ROSLaunchParent(self.uuid, view_roslaunch_file)


        # Start the camera driver
        try:
            cam_driver.start()
            self.running_processes[f'{camera_name}_driver'] = cam_driver
            if view_camera:
                view_output.start()
                self.running_processes[f'{camera_name}_view'] = view_output

        except roslaunch.RLException as excep_camera:
            rospy.logerr(f"Error starting {camera_name} camera driver: {str(excep_camera)}")
            return

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

    def stop_camera(self, camera_name):
        """Stop a camera driver."""
        # Check if camera driver is running
        if f'{camera_name}_driver' not in self.running_processes:
            rospy.logwarn(f"{camera_name} camera driver is not running.")
            return

        # Shutdown the camera driver
        try:
            self.running_processes[f'{camera_name}_driver'].shutdown()
            self.running_processes[f'{camera_name}_view'].shutdown()
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
        # Print success message
        rospy.loginfo(f"{camera_name} camera driver stopped successfully.")

    def sidebar_btn_cam_1_start_event(self):
        """This function is called when camera 1 button is pressed"""
        camera_name = 'camera_1'
        device_id = 0
        calibration_file = 'cam1'

        # Start or stop the camera depending on its current state
        if self.camera_1_active is False:
            # Start camera 1
            try:
                self.start_camera(camera_name, device_id, calibration_file, view_camera=True)
            except Exception as excep_camera:
                rospy.logerr(f"Error starting {camera_name} camera: {str(excep_camera)}")
                self.camera_1_active = False
                return

            # Update button text and color
            self.sidebar_btn_cam_1_start.configure(text="Stop Camera 1", fg_color=("#fa5f5a", "#ba3732"))
        else:
            # Stop camera 1
            try:
                self.stop_camera(camera_name)
            except Exception as e:
                rospy.logerr(f"Error stopping {camera_name} camera: {str(e)}")
                self.camera_1_active = True
                return
            self.sidebar_btn_cam_1_start.configure(text="Start Camera 1", fg_color=themes[color_select])

    def sidebar_btn_cam_2_start_event(self):
        """This function is called when camera 2 button is pressed"""
        camera_name = 'camera_2'
        device_id = 1
        calibration_file = 'cam2'

        # Start or stop the camera depending on its current state
        if self.camera_2_active is False:
            # Start camera 2
            try:
                self.start_camera(camera_name, device_id, calibration_file, view_camera=True)
            except Exception as e:
                rospy.logerr(f"Error starting {camera_name} camera: {str(e)}")
                self.camera_2_active = False
                return

            # Update button text and color
            self.sidebar_btn_cam_2_start.configure(text="Stop Camera 2", fg_color=("#fa5f5a", "#ba3732"))
        else:
            # Stop camera 2
            try:
                self.stop_camera(camera_name)
            except Exception as e:
                rospy.logerr(f"Error stopping {camera_name} camera: {str(e)}")
                self.camera_2_active = True
                return
            self.sidebar_btn_cam_2_start.configure(text="Start Camera 2", fg_color=themes[color_select])

    def sidebar_btn_cam_3_start_event(self):
        """This function is called when camera 3 button is pressed"""
        camera_name = 'camera_3'
        device_id = 2
        calibration_file = 'cam3'

        # Start or stop the camera depending on its current state
        if self.camera_3_active is False:
            # Start camera 3
            try:
                self.start_camera(camera_name, device_id, calibration_file, view_camera=True)
            except Exception as e:
                rospy.logerr(f"Error starting {camera_name} camera: {str(e)}")
                self.camera_3_active = False
                return

            # Update button text and color
            self.sidebar_btn_cam_3_start.configure(text="Stop Camera 3", fg_color=("#fa5f5a", "#ba3732"))
        else:
            # Stop camera 3
            try:
                self.stop_camera(camera_name)
            except Exception as e:
                rospy.logerr(f"Error stopping {camera_name} camera: {str(e)}")
                self.camera_3_active = True
                return
            self.sidebar_btn_cam_3_start.configure(text="Start Camera 3", fg_color=themes[color_select])


    def sidebar_button_event(self, camera_num):
        self.launch_files = [
            f"{self.launch_path}calib{num}.launch" for num in range(1, 4)]
        self.calibrations = {f"cam{num}_calib": roslaunch.parent.ROSLaunchParent(
            self.uuid, [self.launch_files[num-1]]) for num in range(1, 4)}

        """Launches the camera calibration process for the camera 1"""
        print("="*36, "      Starting Camera Calibration", "="*36, sep="\n")
        # Launch parent processes for each camera calibration

        # Start calibration process and wait for it to finish
        calibration = self.calibrations[f"cam{camera_num}_calib"]
        calibration.start()
        self.running_processes.update({f"cam{camera_num}_calib": calibration})
        while calibration.pm.is_alive():
            time.sleep(1)
        calibration.shutdown()
        del self.running_processes[f"cam{camera_num}_calib"]

    def exit_button_click(self):
        print("Terminated successfully.")
        self.destroy()
        exit()


if __name__ == "__main__":
    app = GUI()
    app.mainloop()
