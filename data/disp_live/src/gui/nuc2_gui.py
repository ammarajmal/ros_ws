#!/usr/bin/env python3
""" backend definitions for the gui"""
import subprocess
import tkinter as tk
import customtkinter
import rospy
import rospkg
import roslaunch
themes = {'blue': ("#3B8ED0", "#1F6AA5"),
          'green': ("#2CC985", "#2FA572"),
          'dark-blue': ("#3a7ebf", "#1f538d")
          }

ACTIVE_CAMERA = 'Camera 2'


# select value of COLOR_SELECT from (0: blue, 1: green, 2: dark-blue)
COLOR_SELECT = list(themes.keys())[2]
print(COLOR_SELECT)

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
        rospy.init_node("nuc2_gui", anonymous=False)
        
        self.package = 'gige_cam_driver'

        # ********************************************************************************
        # Path management for Launch files
        # ********************************************************************************

        self.launch_path = rospkg.RosPack().get_path('gige_cam_driver') + '/launch/'
        self.csv_folder_path = rospkg.RosPack().get_path('gige_cam_driver') + '/csvfiles/'
        self.detect_launch_path = rospkg.RosPack().get_path('aruco_detect') + '/launch/'
        self.bagfile_path = self.launch_path.replace("launch/", "bagfiles/")

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(self.uuid)
        self.cam_launch = f"{self.launch_path}cam.launch"
        self.view_launch = f"{self.launch_path}viewcam.launch"
        self.calib_launch = f"{self.launch_path}calib.launch"


        
        
        self.title("NUC2 Dashboard")
        self.geometry("1000x600")
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.destroy_routine)

        # Create a BooleanVar to use as the variable for the checkbox
        self.view_camera = tk.BooleanVar()
        self.view_camera.set(False)  # Set the initial value to False
        
        self._create_widgets()
        self.nuc2_camera_active = False
        self.camera_2_active = False
        self.camera_3_active = False
        self.running_processes = {}
        self.left_frame = None
        self.left_top_frame = None

        self.left_top_frame_label = None
        self.left_top_frame_button = None
        self.left_bottom_frame = None
        self.left_bottom_frame_label = None
        self.left_bottom_frame_calib1_button = None
        self.left_bottom_frame_calib2_button = None
        self.left_bottom_frame_calib3_button = None
        self.left_bottom_frame_sq_size_label = None
        self.left_bottom_frame_sq_size_entry = None
        self.right_top_frame_label = None
        self.left_top_frame_view_cam_checkbox = None
        # self.left_top_frame_nuc2_cam_button = customtkinter.CTkButton(self.left_top_frame)
        
        
        self.left_bottom_frame_chessboard_label = None
        self.left_bottom_frame_chessboard_entry = None
        self.left_button_frame_calib_update_button = None
        self.right_frame = None
        self.right_top_frame = None
        self.right_top_frame_ros_status_label = None
        self.right_top_frame_ros_status_result_label = None
        self.right_top_frame_camera_label = None
        self.right_top_frame_camera_result_label = None
        self.right_bottom_frame = None
        

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
        self._create_left_bottom_frame()
    def _create_left_top_frame(self) -> None:
        """_summary_
        """
        self.left_top_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_top_frame.place(relx=0.1, rely=0.03, relwidth=.8, relheight=0.35)
        self._create_left_top_frame_content()
    def _create_left_top_frame_content(self)-> None:
        """_summary_
        """
        self.left_top_frame_label = customtkinter.CTkLabel(
            self.left_top_frame, text="CAMERA OUTPUT")
        self.left_top_frame_label.place(relx=0.5, rely=0.10, anchor="center")

        self.left_top_frame_nuc2_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start Camera 1",
            fg_color=themes[COLOR_SELECT][1],
            command=lambda: self._start_cam_1_button_event(1, True, False))
        self.left_top_frame_nuc2_cam_button.place(relx=0.5, rely=0.25, anchor="center")
        
        self.left_top_frame_cam2_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start Camera 2",
            fg_color=themes[COLOR_SELECT][1],
            command=lambda: self._start_cam_1_button_event(2, True, False))
        self.left_top_frame_cam2_button.place(relx=0.5, rely=0.45, anchor="center")

        self.left_top_frame_cam3_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start Camera 3",
            fg_color=themes[COLOR_SELECT][1],
            command=lambda: self._start_cam_1_button_event(3, True, False))
        self.left_top_frame_cam3_button.place(relx=0.5, rely=0.65, anchor="center")
                
        self.left_top_frame_view_cam_checkbox = customtkinter.CTkCheckBox(
            self.left_top_frame, text="Enable View",
            checkbox_height=17, checkbox_width=17,
            onvalue=True, offvalue=False, variable=self.view_camera,
            command=self._view_camera_checkbox_event)
        self.left_top_frame_view_cam_checkbox.place(relx=0.5, rely=0.85, anchor="center")
        

    
    def _start_cam_1_button_event(self, camera_number, show_camera, calibrate_camera) -> None:
        """This function is used to start the camera node"""
        print("Starting Camera Node..")

        cameras = [
        {'camera_name': 'nuc2_camera',
         'device_id': 0,
         'calibration_file': 'nuc2_cam',
         'button': self.left_top_frame_nuc2_cam_button,
        #  'calibrate_button': self.sidebar_btn_cam_1_calib,
         'name': 'Camera 1'},
        {'camera_name': 'camera_2',
         'device_id': 1,
         'calibration_file': 'cam2',
        #  'button': self.sidebar_btn_cam_2_start,
        #  'calibrate_button': self.sidebar_btn_cam_2_calib,
         'name': 'Camera 2'},
        {'camera_name': 'camera_3',
         'device_id': 2,
         'calibration_file': 'cam3',
        #  'button': self.sidebar_btn_cam_3_start,
        #  'calibrate_button': self.sidebar_btn_cam_3_calib,
         'name': 'Camera 3'}
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
        camera_active_states = [self.nuc2_camera_active,
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
        self.nuc2_camera_active, self.camera_2_active, self.camera_3_active = camera_active_states
    def start_camera(self, camera_name, device_id, calibration_file, view_camera, calibrate_camera) -> None:
        """Starts a camera driver and optionally a camera view"""
        print("Camera Node Started...")
            # Check if the camera driver is already running
        if f'{camera_name}_driver' in self.running_processes:
            rospy.logwarn(f"{camera_name} camera driver is already running.")
            return
        camera_launch_args = [f"{self.cam_launch}",
                              f"cam:={camera_name}",
                              f"device_id:={device_id}",
                              f"calib_file:={calibration_file}"
                              ]
        view_launch_args = [self.view_launch,
                            f"camera_name:={camera_name}"
                            ]


        # Create a ROS launch file with the camera launch command
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(
            camera_launch_args)[0], camera_launch_args[1:])]
        cam_driver = roslaunch.parent.ROSLaunchParent(
            self.uuid, roslaunch_file)

        # Start the camera driver
        try:
            rospy.sleep(0.5)
            cam_driver.start()
            self.running_processes[f'{camera_name}_driver'] = cam_driver

            # Set camera active flag to True
            if camera_name == 'nuc2_camera':
                self.nuc2_camera_active = True
            elif camera_name == 'camera_2':
                self.camera_2_active = True
            elif camera_name == 'camera_3':
                self.camera_3_active = True

            # Print success message
            rospy.loginfo(f"{camera_name} camera driver started successfully.")
            rospy.sleep(0.5)

            # If view_camera is True, start the camera view
            if view_camera:
                view_launch_file = [(roslaunch.rlutil.resolve_launch_arguments(
                    view_launch_args)[0], view_launch_args[1:])]
                view_output = roslaunch.parent.ROSLaunchParent(
                    self.uuid, view_launch_file)
                view_output.start()
                self.running_processes[f'{camera_name}_view'] = view_output


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
        if camera_name == 'nuc2_camera':
            self.nuc2_camera_active = False
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
                
    def _view_camera_checkbox_event(self) -> None:
        """_summary_
        """
        print(self.view_camera.get())
        if self.view_camera.get():
            self.left_top_frame_nuc2_cam_button.configure(text="View Camera 1", fg_color=themes[COLOR_SELECT][0])
        else:
            self.left_top_frame_nuc2_cam_button.configure(text="Start Camera 1",
                                                            fg_color=themes[COLOR_SELECT][1])
    def _create_left_bottom_frame(self) -> None:
        """_summary_
        """
        self.left_bottom_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_bottom_frame.place(relx=0.1, rely=0.41, relwidth=0.8, relheight=0.55)
        self._create_left_bottom_frame_content()
    def _create_left_bottom_frame_content(self) -> None:
        """_summary_
        """
        self.left_bottom_frame_label = customtkinter.CTkLabel(
            self.left_bottom_frame, text="CAMERA CALIBRATION")
        self.left_bottom_frame_label.grid(row=0, column=0, padx=10, pady=(10,0), sticky="nsew")

        self.left_bottom_frame_calib1_button = customtkinter.CTkButton(
            self.left_bottom_frame,
            text="Camera 1 Calibration",
            state=("normal" if ACTIVE_CAMERA == "Camera 1" else "disabled"),
            fg_color=(None if ACTIVE_CAMERA == "Camera 1"  else "gray")
        )
        self.left_bottom_frame_calib1_button.grid(row=1, column=0, padx=35, pady=(10,0), sticky="nsew")

        self.left_bottom_frame_calib2_button = customtkinter.CTkButton(
            self.left_bottom_frame,
            text="Camera 2 Calibration",
            state=("normal" if ACTIVE_CAMERA == "Camera 2" else "disabled"),
            fg_color=(None if ACTIVE_CAMERA == "Camera 2" else "gray")
        )
        self.left_bottom_frame_calib2_button.grid(row=2, column=0, padx=35, pady=(10,0), sticky="nsew")

        self.left_bottom_frame_calib3_button = customtkinter.CTkButton(
            self.left_bottom_frame,
            text="Camera 3 Calibration",
            state=("normal" if ACTIVE_CAMERA == "Camera 3" else "disabled"),
            fg_color=(None if ACTIVE_CAMERA == "Camera 3" else "gray")
        )
        self.left_bottom_frame_calib3_button.grid(row=3, column=0, padx=35, pady=(10,0), sticky="nsew")



        self.left_bottom_frame_sq_size_label = customtkinter.CTkLabel(
            self.left_bottom_frame, text="Square Size")
        self.left_bottom_frame_sq_size_label.grid(row=4, column=0, padx=35, pady=(10,0), sticky="nsew")

        self.left_bottom_frame_sq_size_entry = customtkinter.CTkEntry(
            self.left_bottom_frame)
        self.left_bottom_frame_sq_size_entry.grid(row=5, column=0, padx=35, pady=(0,0), sticky="nsew")

        self.left_bottom_frame_chessboard_label = customtkinter.CTkLabel(
            self.left_bottom_frame, text="Chessboard Size")
        self.left_bottom_frame_chessboard_label.grid(row=6, column=0, padx=35, pady=(0,0), sticky="nsew")

        self.left_bottom_frame_chessboard_entry = customtkinter.CTkEntry(
            self.left_bottom_frame)
        self.left_bottom_frame_chessboard_entry.grid(row=7, column=0, padx=35, pady=(0,0), sticky="nsew")
        
        self.left_button_frame_calib_update_button = customtkinter.CTkButton(
            self.left_bottom_frame, text="Update", fg_color="green")
        self.left_button_frame_calib_update_button.grid(row=8, column=0, padx=35, pady=(10,0), sticky="nsew")
        
        self.left_bottom_frame_calib1_button.bind(
            "<Button-1>", lambda event: self._calibrate_camera(1))
        
        
    
        
        
    def _create_right_frame(self) -> None:
        """_summary_
        """
        self.right_frame = tk.Frame(self, bg=themes[COLOR_SELECT][1])
        self.right_frame.place(relx=0.25, rely=0, relwidth=0.75, relheight=1)
        self._create_right_top_frame()
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
            self.right_top_frame, text=" NUC2 ", text_color="yellow", bg_color=themes[COLOR_SELECT][1])
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
            self.right_top_frame, text=ACTIVE_CAMERA, text_color="white")
        self.right_top_frame_camera_result_label.place(relx=0.72, rely=0.5, anchor="center")
        
        
        
    def _create_right_bottom_frame(self) -> None:
        """_summary_
        """
        self.right_bottom_frame = tk.Frame(self.right_frame, bg=themes[COLOR_SELECT][0])
        self.right_bottom_frame.place(relx=.01, rely=0.14, relwidth=.94, relheight=0.79)
if __name__ == "__main__":
    root = ClientGUI()
    root.mainloop()
    