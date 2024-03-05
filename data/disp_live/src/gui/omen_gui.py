#!/usr/bin/env python3
""" backend definitions for the gui"""
# import subprocess
import tkinter as tk
import customtkinter
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
        rospy.init_node("main_gui", anonymous=False)
        
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
        self.nuc1_launch = f"{self.launch_path}nuc1_remote_cam.launch"
        self.nuc2_launch = f"{self.launch_path}nuc2_remote_cam.launch"
        self.nuc3_launch = f"{self.launch_path}nuc3_remote_cam.launch"
        self.view_launch = f"{self.launch_path}viewcam.launch"
        self.calib_launch = f"{self.launch_path}calib.launch"


        
        
        self.title("Main Dashboard")
        self.geometry("1000x600")
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.destroy_routine)

        # Create a BooleanVar to use as the variable for the checkbox
        self.view_camera = tk.BooleanVar()
        self.view_camera.set(False)  # Set the initial value to False
        
        self._create_widgets()
        self.camera_1_active = False
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
        # self.left_top_frame_cam1_button = customtkinter.CTkButton(self.left_top_frame)
        self.right_top_frame_system_label = None
        
        
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
            self.left_top_frame, text="START REMOTE CAMERA")
        self.left_top_frame_label.place(relx=0.5, rely=0.15, anchor="center")

        self.left_top_frame_cam1_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start Camera - NUC 1",
            fg_color=themes[COLOR_SELECT][1],
            command=lambda: self._start_nuc_cam_button_event(1, True, False))
        self.left_top_frame_cam1_button.place(relx=0.5, rely=0.35, anchor="center")
        
        self.left_top_frame_cam2_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start Camera - NUC 2",
            fg_color=themes[COLOR_SELECT][1],
            command=lambda: self._start_nuc_cam_button_event(2, True, False))
        self.left_top_frame_cam2_button.place(relx=0.5, rely=0.57, anchor="center")

        self.left_top_frame_cam3_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start Camera - NUC 3",
            fg_color=themes[COLOR_SELECT][1],
            command=lambda: self._start_nuc_cam_button_event(3, True, False))
        self.left_top_frame_cam3_button.place(relx=0.5, rely=0.80, anchor="center")
    
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
if __name__ == "__main__":
    root = ClientGUI()
    root.mainloop()
    