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

color_select = list(themes.keys())[1]

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

        # configure window
        self.title("Displacement Measurement using ARUCO Marker")
        self.geometry('970x670')

        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure((1, 2), weight=0)

        self.grid_rowconfigure((0, 1, 2), weight=1)
        # create sidebar frame with widgets
        self.sidebar_frame = customtkinter.CTkFrame(
            master=self,
            width=140,
            corner_radius=0
        )
        self.sidebar_frame.grid(row=0, column=0, rowspan=11, sticky="nsew")

        self.sidebar_label = customtkinter.CTkLabel(
            master=self.sidebar_frame,
            text="Camera Calibration",
            font=customtkinter.CTkFont(size=20, weight="bold")
        )
        self.sidebar_label.grid(row=0, column=0, padx=20, pady=(20, 10))

        self.sidebar_button_1 = customtkinter.CTkButton(
            master=self.sidebar_frame,
            text='Calibrate Camera 1',
            command=lambda: self.sidebar_button_event(1)
        )
        self.sidebar_button_2 = customtkinter.CTkButton(
            master=self.sidebar_frame,
            text='Calibrate Camera 2',
            command=lambda: self.sidebar_button_event(2)
        )
        self.sidebar_button_3 = customtkinter.CTkButton(
            self.sidebar_frame,
            text='Calibrate Camera 3',
            command=lambda: self.sidebar_button_event(3)
        )
        self.sidebar_button_1.grid(row=1, column=0, padx=20, pady=10)
        self.sidebar_button_2.grid(row=2, column=0, padx=20, pady=10)
        self.sidebar_button_3.grid(row=3, column=0, padx=20, pady=10)

        self.sidebar_button_4_label = customtkinter.CTkLabel(
            master=self.sidebar_frame,
            text="Camera testing:",
            anchor="w"
        )
        self.sidebar_button_4_label.grid(row=4, column=0, padx=20, pady=(10, 0))

        self.sidebar_button_4 = customtkinter.CTkButton(
            master=self.sidebar_frame,
            text='Start Camera 1',
            command=self.camera_button_event_1
        )
        self.sidebar_button_5 = customtkinter.CTkButton(
            master=self.sidebar_frame,
            text='Start Camera 2',
            command=self.camera_button_event_2
        )
        self.sidebar_button_6 = customtkinter.CTkButton(
            master=self.sidebar_frame,
            text='Start Camera 3',
            command=self.camera_button_event_3
        )
        self.sidebar_button_4.grid(row=5, column=0, padx=20, pady=10)
        self.sidebar_button_5.grid(row=6, column=0, padx=20, pady=10)
        self.sidebar_button_6.grid(row=7, column=0, padx=20, pady=10)

        self.appearance_mode_label = customtkinter.CTkLabel(
            master=self.sidebar_frame,
            text="Appearance Mode:",
            anchor="w"
        )
        self.appearance_mode_label.grid(row=8, column=0, padx=20, pady=(10, 0))

        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(
            master=self.sidebar_frame,
            values=["Light", "Dark", "System"],
            command=self.change_appearance_mode_event
        )
        self.appearance_mode_optionemenu.grid(row=9, column=0, padx=20, pady=(10, 10))

        self.scaling_label = customtkinter.CTkLabel(
            master=self.sidebar_frame,
            text="UI Scaling:",
            anchor="w"
        )
        self.scaling_label.grid(row=10, column=0, padx=20, pady=(10, 0))

        self.scaling_optionemenu = customtkinter.CTkOptionMenu(
            master=self.sidebar_frame,
            values=["80%", "90%", "100%", "110%", "200%"],
            command=self.change_scaling_event
        )
        self.scaling_optionemenu.grid(row=11, column=0, padx=20, pady=(10, 20))

        self.main_label = customtkinter.CTkLabel(
            master=self,
            text="Main Processing",
            font=customtkinter.CTkFont(size=20, weight="bold")
        )
        self.main_label.place_configure(relx=0.64, rely=0.055, anchor='center')

        self.main_button_exit = customtkinter.CTkButton(
            master=self,
            text='EXIT',
            command=self.exit_button_click,
            fg_color="transparent",
            border_width=2,
            text_color=("gray10", "#DCE4EE")
        )
        # self.main_button_exit.grid(row=2, column=1, padx=(20, 20), pady=(20, 20), sticky="nsew")
        self.main_button_exit.place_configure(relx=0.64, rely=0.93, anchor='center')

        # self.main_label.place(relx=0.6, rely=0.055, anchor='center')

        self.tabview = customtkinter.CTkTabview(self)
        self.tabview.place_configure(
            x=250,
            y=50,
            relwidth=0.7,
            relheight=0.8
        )
        # self.tabview.grid(row=1, column=1, padx=(
            # 20, 0), pady=(50, 20), sticky="nsew")
        self.tabview.add("Recording Data")
        self.tabview.add("Post-Processing")
        self.tabview.add("Displaying Results")
        self.tabview.tab("Recording Data").grid_columnconfigure(0, weight=1)  # configure grid of individual tabs
        self.tabview.tab("Post-Processing").grid_columnconfigure(0, weight=1)


        self.record_label = customtkinter.CTkLabel(
            master=self.tabview.tab("Recording Data"),
            text="Recording from Camera",
            font=customtkinter.CTkFont(size=16)
        )
        self.record_label.grid(row=0, column=0, padx=20, pady=(5, 0), sticky="nsew")

        self.record_single_frame = customtkinter.CTkFrame(
            master=self.tabview.tab("Recording Data"),
            # bg_color="red"  # set background color to red
        )
        self.record_multiple_frame = customtkinter.CTkFrame(
            master=self.tabview.tab("Recording Data"),
            # bg_color="blue"  # set background color to blue
        )
        self.single_cam_label = customtkinter.CTkLabel(
            master=self.record_single_frame,
            text="Single camera",
            font=customtkinter.CTkFont(size=16),
            text_color="#505050"
        )
        self.single_cam_select_label = customtkinter.CTkLabel(
            master=self.record_single_frame,
            text="Select a Camera:",
            font=customtkinter.CTkFont(size=14),
            text_color="#707070"
        )
        self.single_dur_select_label = customtkinter.CTkLabel(
            master=self.record_single_frame,
            text="Select duration:",
            font=customtkinter.CTkFont(size=14),
            text_color="#707070"
        )
        self.single_rec_manual_label = customtkinter.CTkLabel(
            master=self.record_single_frame,
            text="Start Recording manually:",
            font=customtkinter.CTkFont(size=14),
            text_color="#707070"
        )
        self.single_dur_select_or_label = customtkinter.CTkLabel(
            master=self.record_single_frame,
            text="or",
            font=customtkinter.CTkFont(size=14),
            text_color="#707070"
        )
 
        self.camera_selection_var = tk.StringVar()

        
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
            text_color="#808080"
        
        )
        self.single_camera_1_radio = customtkinter.CTkRadioButton(
                    master=self.record_single_frame,
            value="Camera 1",
            text="Camera 1",
            font=customtkinter.CTkFont(size=14),
            command=self.checkbox_event,
            variable=self.camera_selection_var,
            text_color="#808080"
            )
        self.single_camera_2_radio = customtkinter.CTkRadioButton(
                    master=self.record_single_frame,
            value="Camera 2",
            text="Camera 2",
            font=customtkinter.CTkFont(size=14),
            command=self.checkbox_event,
            variable=self.camera_selection_var,
            text_color="#808080"
            )
        self.single_camera_3_radio = customtkinter.CTkRadioButton(
                    master=self.record_single_frame,
            value="Camera 3",
            text="Camera 3",
            font=customtkinter.CTkFont(size=14),
            command=self.checkbox_event,
            variable=self.camera_selection_var,
            text_color="#808080"
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
            dropdown_text_color="#808080",
            dropdown_fg_color='#ffffff',
            text_color="#303030"
            )
        self.single_camera_dur_entry = customtkinter.CTkEntry(
            master=self.record_single_frame,
            placeholder_text="Manual entry",
            placeholder_text_color="#808080",
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
            text_color="#505050"
        )
        self.multi_cam_select_label = customtkinter.CTkLabel(
            master=self.record_multiple_frame,
            text="Select Cameras:",
            font=customtkinter.CTkFont(size=14),
            text_color="#707070"
        )
        self.multi_dur_select_label = customtkinter.CTkLabel(
            master=self.record_multiple_frame,
            text="Select duration:",
            font=customtkinter.CTkFont(size=14),
            text_color="#707070"
        )
        self.multi_rec_manual_label = customtkinter.CTkLabel(
            master=self.record_multiple_frame,
            text="Start Recording manually:",
            font=customtkinter.CTkFont(size=14),
            text_color="#707070"
        )
        self.multi_dur_select_or_label = customtkinter.CTkLabel(
            master=self.record_multiple_frame,
            text="or",
            font=customtkinter.CTkFont(size=14),
            text_color="#707070"
        )
 
        self.check_camera_1_var = tk.StringVar(self, "on")
        self.check_camera_2_var = tk.StringVar(self, "on")
        self.check_camera_3_var = tk.StringVar(self, "on")
 
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
            text_color="#808080"
        
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
            text_color="#808080"
        
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
            text_color="#808080"
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
            dropdown_text_color="#808080",
            dropdown_fg_color='#ffffff',
            text_color="#303030"
            )
        self.multi_camera_dur_entry = customtkinter.CTkEntry(
            master=self.record_multiple_frame,
            placeholder_text="Manual entry",
            placeholder_text_color="#808080",
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

    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 100
        customtkinter.set_widget_scaling(new_scaling_float)

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

    def camera_button_event_1(self):
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
            self.sidebar_button_4.configure(text="Stop Camera 1", fg_color=("#fa5f5a", "#ba3732"))
        else:
            # Stop camera 1
            try:
                self.stop_camera(camera_name)
            except Exception as e:
                rospy.logerr(f"Error stopping {camera_name} camera: {str(e)}")
                self.camera_1_active = True
                return
            self.sidebar_button_4.configure(text="Start Camera 1", fg_color=themes[color_select])

    def camera_button_event_2(self):
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
            self.sidebar_button_5.configure(text="Stop Camera 2", fg_color=("#fa5f5a", "#ba3732"))
        else:
            # Stop camera 2
            try:
                self.stop_camera(camera_name)
            except Exception as e:
                rospy.logerr(f"Error stopping {camera_name} camera: {str(e)}")
                self.camera_2_active = True
                return
            self.sidebar_button_5.configure(text="Start Camera 2", fg_color=themes[color_select])

    def camera_button_event_3(self):
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
            self.sidebar_button_6.configure(text="Stop Camera 3", fg_color=("#fa5f5a", "#ba3732"))
        else:
            # Stop camera 3
            try:
                self.stop_camera(camera_name)
            except Exception as e:
                rospy.logerr(f"Error stopping {camera_name} camera: {str(e)}")
                self.camera_3_active = True
                return
            self.sidebar_button_6.configure(text="Start Camera 3", fg_color=themes[color_select])


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
