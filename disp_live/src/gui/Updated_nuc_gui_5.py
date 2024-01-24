#!/usr/bin/env python3
""" backend definitions for the gui"""

import subprocess
import tkinter as tk
import customtkinter
import rospy
import rospkg
import roslaunch
from fiducial_msgs.msg import FiducialTransformArray
from fast_cam.msg import CameraSpecs
from fast_cam.srv import SetGain, GetCameraProperties
from _backend_ import is_node_running, kill_ros_node, detection_start, detection_stop, get_ros_topic_frequency
themes = {'blue': ("#3B8ED0", "#1F6AA5"),
          'green': ("#2CC985", "#2FA572"),
          'dark-blue': ("#3a7ebf", "#1f538d"),
          'red': ("#fa5f5a", "#ba3732")
          }

# select value of COLOR_SELECT from (0: blue, 1: green, 2: dark-blue)
COLOR_SELECT = list(themes.keys())[2]

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
        self.nuc_number = '5'

        rospy.init_node(f"camera_{self.nuc_number}_gui", anonymous=False)
        # rospy.wait_for_service(f"/camera_{self.nuc_number}/set_gain")
        # self.set_gain = rospy.ServiceProxy(f"/camera_{self.nuc_number}/set_gain", SetGain)

        self.message_timeout = 0.5  # 0.5 second
        self.timeout_timer = None
        self.package = 'fast_cam'
        # ********************************************************************************
        # Path management for Launch files
        # ********************************************************************************
        self.launch_path = rospkg.RosPack().get_path(self.package) + '/launch/'
        self.prev_cam_path = rospkg.RosPack().get_path('gige_cam_driver') + '/launch/'
        self.csv_folder_path = rospkg.RosPack().get_path('gige_cam_driver') + '/csvfiles/'
        self.detect_launch_path = rospkg.RosPack().get_path('aruco_detect') + '/launch/'

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.local_nuc_launch = f'{self.launch_path}use.launch'
        self.view_launch = f"{self.prev_cam_path}viewcam.launch"
        self.calib_launch = f"{self.prev_cam_path}calib.launch"
        self.detect_launch = f"{self.detect_launch_path}local_detect_cam.launch"

        self.title(f"CAMERA {self.nuc_number} Dashboard")
        self.geometry("960x500")
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.destroy_routine)
        self.view_camera = tk.BooleanVar()
        self.view_camera.set(False)  # Set the initial value to False
        # self.nuc1_camera = False
        # self.nuc2_camera = False
        # self.nuc3_camera = False
        self.board_size = "4x4"  # dimensions for calibration
        self.square_size = "0.01725"  # in meters for calibrationclear
        self.marker_dim = "0.0200"  # in meters for ARUCO marker
        self.marker_dict = "0"  # (DICT_4X4_50)
        self.var_dictionary = tk.StringVar(self, "0")  # dict 5x5 (1000)
        self.running_processes = {}
        self.left_frame = None
        self.left_top_frame = None
        self.left_top_frame_label = None
        self.left_top_frame_button = None
        self._create_left_exit_button()
        self.middle_center_frame = None
        self.middle_center_frame_label = None
        self.middle_center_frame_start_calib_button = customtkinter.CTkButton(
            master=self.middle_center_frame)
        self.middle_center_frame_sq_size_label = None
        self.middle_center_frame_sq_size_entry = None
        self.right_top_frame_system_label = None
        self.right_top_frame_label = None
        self.left_top_frame_view_cam_checkbox = None
        self.middle_center_frame_chessboard_label = None
        self.middle_center_frame_chessboard_entry = None
        self.left_button_frame_calib_update_button = customtkinter.CTkButton(
            master=self.middle_center_frame)
        self.middle_frame = None
        self.right_frame = None
        
        self.right_top_frame = None
        self.right_top_frame_ros_status_label = None
        self.right_top_frame_ros_status_result_label = None
        self.right_top_frame_camera_label = None
        self.right_top_frame_camera_result_label = None
        self.right_bottom_frame = None
        self.right_bottom_frame_detect_result_ans_label = tk.Label(
            self, text="Initializing...")
        self.right_bottom_frame_detect_result_ans_label.pack()
        self.left_bottom_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_bottom_frame_label = customtkinter.CTkLabel(
            self.left_bottom_frame)
        self.left_button_frame_calib_update_label = customtkinter.CTkLabel(
            self.middle_center_frame)
        self.left_bottom_frame_start_local_detect_button = customtkinter.CTkButton(
            self.left_bottom_frame)
        self.left_bottom_frame_start_nuc2_cam_button = customtkinter.CTkButton(
            self.left_bottom_frame)
        self.left_bottom_frame_start_nuc3_cam_button = customtkinter.CTkButton(
            self.left_bottom_frame)
        self._create_widgets()

    # def start_ros_thread(self) -> None:
    #     # Existing fiducial transforms listener thread
    #     threading.Thread(target=self.fiducial_transforms_listener, daemon=True).start()

        # New camera specifications listener thread
        # threading.Thread(target=self.camera_specs_listener,daemon=True).start()

    # def camera_specs_listener(self):
    #     rospy.Subscriber(
    #         f"/camera_{self.nuc_number}/camera_specifications", CameraSpecs, self.camera_specs_callback)
    #     rospy.spin()

    # def camera_specs_callback(self, data):
    #     """ Update the camera specs """
    #     self.right_bottom_frame_cam_name_result_label.configure(
    #         text=f"Camera {self.nuc_number}", text_color="white")
    #     self.right_bottom_frame_cam_model_result_label.configure(
    #         text=data.model, text_color="white")
    #     self.right_bottom_frame_cam_serial_result_label.configure(
    #         text=data.serial_number, text_color="white")
    #     self.right_bottom_frame_cam_ip_result_label.configure(
    #         text=data.ip_address, text_color="white")
    #     self.right_bottom_frame_cam_reolution_result_label.configure(
    #         text=data.resolution, text_color="white")
        # self.right_bottom_frame_cam_exposure_result_label.configure(text=data.gain, text_color="white")

    #     if hasattr(self, 'camera_specs_timer') and self.camera_specs_timer:
    #         self.camera_specs_timer.cancel()
    #     self.camera_specs_timer = threading.Timer(
    #         self.message_timeout, self.camera_specs_no_data)
    #     self.camera_specs_timer.start()

    # def camera_specs_no_data(self):
    #     """ Function to call when no messages are received within the timeout """
    #     # self.right_bottom_frame_cam_name_result_label.configure(text="Null", text_color="white")
    #     self.right_bottom_frame_cam_model_result_label.configure(
    #         text="-", text_color="white")
    #     self.right_bottom_frame_cam_serial_result_label.configure(
    #         text="-", text_color="white")
    #     self.right_bottom_frame_cam_ip_result_label.configure(
    #         text="-", text_color="white")
    #     self.right_bottom_frame_cam_reolution_result_label.configure(
    #         text="-", text_color="white")

    # def fiducial_transforms_listener(self):
    #     rospy.Subscriber(f"/camera_{self.nuc_number}/fiducial_transforms",
    #                      FiducialTransformArray, self.fiducial_transforms_callback)
    #     rospy.spin()

    # def fiducial_transforms_callback(self, data):
    #     # Reset the timer whenever a message is received
    #     if self.timeout_timer:
    #         self.timeout_timer.cancel()
    #     self.timeout_timer = threading.Timer(self.message_timeout, self.no_message_received)
    #     self.timeout_timer.start()

    #     detected = any(
    #         transform.fiducial_id is not None for transform in data.transforms)
    #     self.right_bottom_frame_detect_result_ans_label.after(
    #         0, lambda: self.update_label(detected))

    # def no_message_received(self):
    #     # Function to call when no messages are received within the timeout
    #     self.right_bottom_frame_detect_result_ans_label.after(
    #         0, lambda: self.update_label(False))

    # def update_label(self, detected):
    #     if detected:
    #         self.right_bottom_frame_detect_result_ans_label.configure(
    #             text="Detected", text_color=themes['green'][0])
    #     else:
    #         self.right_bottom_frame_detect_result_ans_label.configure(
    #             text="Not Detected", text_color="white")

    def destroy_routine(self) -> None:
        """_summary_"""
        self.destroy()
        self.quit()

    def _create_widgets(self) -> None:
        """Starting point of the GUI"""
        self._create_left_frame()
        self._create_middle_frame()
        self._create_right_frame()

    def _create_left_frame(self) -> None:
        """ Camera Settings Frame """
        self.left_frame = tk.Frame(self, bg=themes[COLOR_SELECT][1])
        self.left_frame.place(relx=0, rely=0, relwidth=0.33, relheight=1)
        self._create_left_top_frame()
        self._create_left_bottom_frame()
        self._create_left_exit_button()

    def _create_middle_frame(self) -> None:
        """ Detection Settings Frame """
        self.middle_frame = tk.Frame(self, bg=themes[COLOR_SELECT][1])
        self.middle_frame.place(relx=0.33, rely=0, relwidth=0.33, relheight=1)
        self._create_middle_top_frame()
        self._create_middle_center_frame()
        self._create_middle_bottom_frame()

    def _create_right_frame(self) -> None:
        """ System Status Frame """
        self.right_frame = tk.Frame(self, bg=themes[COLOR_SELECT][1])
        self.right_frame.place(relx=0.66, rely=0, relwidth=0.34, relheight=1)
        self._create_right_top_frame()
        self._create_right_bottom_frame()

    def _create_left_top_frame(self) -> None:
        """ routine to create the top frame of the left panel of GUI"""
        self.left_top_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_top_frame.place(
            relx=0.12, rely=0.06, relwidth=0.8, relheight=0.38)
        self._create_left_top_frame_content()
    def _create_left_bottom_frame(self) -> None:
        """ routine to create the middle frame of the left panel of GUI """
        self.left_bottom_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_bottom_frame.place(
            relx=0.12, rely=0.50, relwidth=0.8, relheight=0.32)
        self._create_left_bottom_frame_content()
    def _create_left_exit_button(self) -> None:
        """ Exit Button """
        self.left_exit_button = customtkinter.CTkButton(self.left_frame, text="Exit", fg_color=themes["red"], command=self.destroy_routine)
        self.left_exit_button.place(relx=0.5, rely=0.90, anchor="center")
    def _create_middle_top_frame(self) -> None:
        """ routine to create the top frame of the left panel of GUI"""
        self.middle_top_frame = customtkinter.CTkFrame(self.middle_frame)
        self.middle_top_frame.place(
            relx=0.06, rely=0.06, relwidth=0.8, relheight=0.08)
        self._create_middle_top_frame_content()
        
    def _create_middle_top_frame_content(self) -> None:
        """ Parameter Update Frame Contents """
        self.middle_top_frame_label = customtkinter.CTkLabel(
            self.middle_top_frame, text=f"UPDATE PARAMETERS - NUC {self.nuc_number}")
        self.middle_top_frame_label.place(relx=0.5, rely=0.5, anchor="center")
    def _create_middle_center_frame(self) -> None:
        """ Camera Calibration Frame """
        self.middle_center_frame = customtkinter.CTkFrame(self.middle_frame)
        self.middle_center_frame.place(
            relx=0.06, rely=0.2, relwidth=0.8, relheight=0.36)
        self._create_middle_center_frame_content()


    def _create_middle_bottom_frame(self) -> None:
        """ routine to create the top frame of the left panel of GUI"""
        self.middle_bottom_frame = customtkinter.CTkFrame(self.middle_frame)
        self.middle_bottom_frame.place(
            relx=0.06, rely=0.62, relwidth=0.8, relheight=0.32)
        self._create_middle_bottom_frame_content()

    def _create_right_top_frame(self) -> None:
        """ SYSTEM IDENTIFICATION FRAME """
        self.right_top_frame = customtkinter.CTkFrame(
            self.right_frame, fg_color=themes[COLOR_SELECT][0])
        self.right_top_frame.place(
            relx=0.0, rely=.06, relwidth=.92, relheight=0.08)
        self._create_right_top_frame_content()

    def _create_right_bottom_frame(self) -> None:
        """ System Data Logging Frame """
        self.right_bottom_frame = customtkinter.CTkFrame(
            self.right_frame, fg_color=themes[COLOR_SELECT][0])
        self.right_bottom_frame.place(
            relx=.0, rely=0.2, relwidth=.92, relheight=0.74)
        self._create_right_bottom_frame_content()

    def _create_right_top_frame_content(self) -> None:
        """System Identification Frame Contents"""
        self.right_top_frame_system_label = customtkinter.CTkLabel(
            self.right_top_frame, text=" CAMERA INFORMATION", text_color="white")
        self.right_top_frame_system_label.place(
            relx=0.50, rely=0.5, anchor="center")
        # self.right_top_frame_label = customtkinter.CTkLabel(
        #     self.right_top_frame, text=f"  Camera {self.nuc_number}  ", text_color="yellow")
        # self.right_top_frame_label.place(relx=0.7, rely=0.5, anchor="center")

    def _create_right_bottom_frame_content(self) -> None:
        """ System Data Logging Frame Contents """
        self.right_bottom_frame_cam_name_label = customtkinter.CTkButton(
            self.right_bottom_frame, text="Name", border_width=1, border_color='white')
        self.right_bottom_frame_cam_name_label.place(relx=0.10, rely=0.03)
        # self.right_bottom_frame_cam_name_result_label = customtkinter.CTkLabel(
        #     self.right_bottom_frame, text=f"Camera {self.nuc_number}", text_color="white")
        self.right_bottom_frame_cam_name_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="-", text_color="white")

        self.right_bottom_frame_cam_name_result_label.place(
            relx=0.6, rely=0.03)

        self.right_bottom_frame_cam_status_label = customtkinter.CTkButton(
            self.right_bottom_frame, text="Status", border_width=1, border_color='white', command=lambda: self._update_camera_specs(self.nuc_number))
        self.right_bottom_frame_cam_status_label.place(relx=0.1, rely=0.12)
        self.right_bottom_frame_cam_status_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="-", text_color="white")
        self.right_bottom_frame_cam_status_result_label.place(
            relx=0.6, rely=0.12)

        self.right_bottom_frame_cam_model_label = customtkinter.CTkButton(
            self.right_bottom_frame, text="Model", border_width=1, border_color='white')
        self.right_bottom_frame_cam_model_label.place(relx=0.1, rely=0.21)
        self.right_bottom_frame_cam_model_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="-", text_color="white")
        self.right_bottom_frame_cam_model_result_label.place(
            relx=0.6, rely=0.21)

        self.right_bottom_frame_cam_serial_label = customtkinter.CTkButton(
            self.right_bottom_frame, text="Serial", border_width=1, border_color='white')
        self.right_bottom_frame_cam_serial_label.place(relx=0.1, rely=0.3)
        self.right_bottom_frame_cam_serial_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="-", text_color="white")
        self.right_bottom_frame_cam_serial_result_label.place(
            relx=0.60, rely=0.3)

        self.right_bottom_frame_cam_ip_label = customtkinter.CTkButton(
            self.right_bottom_frame, text="IP Address", border_width=1, border_color='white')
        self.right_bottom_frame_cam_ip_label.place(relx=0.1, rely=0.39)
        self.right_bottom_frame_cam_ip_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="-", text_color="white")
        self.right_bottom_frame_cam_ip_result_label.place(relx=0.6, rely=0.39)

        self.right_bottom_frame_cam_reolution_label = customtkinter.CTkButton(
            self.right_bottom_frame, text="Resolution", border_width=1, border_color='white')
        self.right_bottom_frame_cam_reolution_label.place(relx=0.1, rely=0.48)
        self.right_bottom_frame_cam_reolution_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="-", text_color="white")
        self.right_bottom_frame_cam_reolution_result_label.place(
            relx=0.6, rely=0.48)

        self.right_bottom_frame_cam_fps_button = customtkinter.CTkButton(
            self.right_bottom_frame, text="Camera FPS", border_width=1, border_color='white', command=lambda: self._check_camera_fps_event(self.nuc_number))
        self.right_bottom_frame_cam_fps_button.place(relx=0.1, rely=0.57)
        self.right_bottom_frame_cam_fps_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="-", text_color="white")
        self.right_bottom_frame_cam_fps_result_label.place(relx=0.6, rely=0.57)

        # self.right_bottom_frame_cam_exposure_label = customtkinter.CTkButton(
        #     self.right_bottom_frame, text="Exposure", border_width=1, border_color='white')
        # self.right_bottom_frame_cam_exposure_label.place(relx=0.1, rely=0.66)
        # self.right_bottom_frame_cam_exposure_result_label = customtkinter.CTkSlider(
        #     self.right_bottom_frame, from_=0, to=50, command=self.on_slider_change,
        #     height=10, number_of_steps=3, width=50, fg_color='white', border_width=1,
        #     border_color='white')
        # self.right_bottom_frame_cam_exposure_result_label.place(
        #     relx=0.6, rely=0.68)

        self.right_bottom_frame_detect_status_label = customtkinter.CTkButton(
            self.right_bottom_frame, text="Detection Status", border_width=1, border_color='white', command=lambda: self._check_detection_event(self.nuc_number),
            fg_color=themes[COLOR_SELECT][1])
        self.right_bottom_frame_detect_status_label.place(relx=0.1, rely=0.66)
        self.right_bottom_frame_detect_status_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="-", text_color="white")
        self.right_bottom_frame_detect_status_label.place(relx=0.6, rely=0.66)
        self.right_bottom_frame_detect_rate_button = customtkinter.CTkButton(
            self.right_bottom_frame, text="Detection Rate", border_width=1, border_color='white', command=lambda: self._check_detection_rate_event(self.nuc_number),
            fg_color=themes[COLOR_SELECT][1])
        self.right_bottom_frame_detect_rate_button.place(relx=0.1, rely=0.75)
        self.right_bottom_frame_detect_rate_result_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="-", text_color="white")
        self.right_bottom_frame_detect_rate_result_label.place(relx=0.6, rely=0.75)

        self.right_bottom_frame_detect_result_label = customtkinter.CTkButton(
            self.right_bottom_frame, text="Detection Result", border_width=1, border_color='white', command=None,
            fg_color=themes[COLOR_SELECT][1])
        self.right_bottom_frame_detect_result_label.place(relx=0.1, rely=0.84)
        self.right_bottom_frame_detect_result_ans_label = customtkinter.CTkLabel(
            self.right_bottom_frame, text="-", text_color="white")
        self.right_bottom_frame_detect_result_ans_label.place(relx=0.6, rely=0.84)

    def _update_camera_specs(self, nuc_number) -> None:
        """ method to check the status of the camera """
        camera_topic_name = f"/camera_{nuc_number}/image_raw"
        if not self.check_active_topic(camera_topic_name):
            # If the camera topic is not active, reset the camera properties
            self.reset_camera_properties_in_gui(nuc_number)
            rospy.logerr(f"Camera at NUC {nuc_number} is not running..")
        else:
            try:
                response = rospy.ServiceProxy(
                    f"/camera_{nuc_number}/get_camera_properties", GetCameraProperties)()
                self.right_bottom_frame_cam_name_result_label.configure(
                    text=f"Camera {nuc_number}", text_color="white")
                self.right_bottom_frame_cam_status_result_label.configure(
                    text="Running", text_color="yellow")
                self.right_bottom_frame_cam_model_result_label.configure(
                    text=response.model, text_color="white")
                self.right_bottom_frame_cam_serial_result_label.configure(
                    text=response.serial_number, text_color="white")
                self.right_bottom_frame_cam_ip_result_label.configure(
                    text=response.ip_address, text_color="white")
                self.right_bottom_frame_cam_reolution_result_label.configure(
                    text=response.resolution, text_color="white")
                # self.right_bottom_frame_cam_exposure_result_label.configure(text=response.gain, text_color="white")
            except rospy.ServiceException as e:
                # Resets the camera properties if the service call fails
                rospy.logerr(f"Service call failed: {e}")
                self.reset_camera_properties_in_gui(nuc_number)
                


    def reset_camera_properties_in_gui(self, nuc_number) -> None:
        """ method to reset the camera properties """
        self.right_bottom_frame_cam_name_result_label.configure(
            text="-", text_color="white")
        self.right_bottom_frame_cam_status_result_label.configure(
            text="-", text_color="white")
        self.right_bottom_frame_cam_model_result_label.configure(
            text="-", text_color="white")
        self.right_bottom_frame_cam_serial_result_label.configure(
            text="-", text_color="white")
        self.right_bottom_frame_cam_ip_result_label.configure(
            text="-", text_color="white")
        self.right_bottom_frame_cam_reolution_result_label.configure(
            text="-", text_color="white")
        # self.right_bottom_frame_cam_exposure_result_label.configure(text="-", text_color="white")
        

    def _check_camera_fps_event(self, nuc_number) -> None:
        """ routine to check the fps of the camera """
        camera_topic_name = f"/camera_{nuc_number}/image_raw"
        if not self.check_active_topic(camera_topic_name):
            self.right_bottom_frame_cam_fps_result_label.configure(
                text="-", text_color="white")
            rospy.logerr(f"Camera at NUC {nuc_number} is not running..")
        else:
            rospy.loginfo(f"Camera at NUC {nuc_number} is running..")
            frequency = get_ros_topic_frequency(camera_topic_name)
            if frequency is not None:
                self.right_bottom_frame_cam_fps_result_label.configure(
                    text=f"{frequency} Hz", text_color="yellow")
                rospy.loginfo(
                    f"Frequency of {camera_topic_name}: {frequency} Hz")
            else:
                self.right_bottom_frame_cam_fps_result_label.configure(
                    text="-", text_color="red")
                rospy.logerr("Unable to determine the frequency.")

    def _check_detection_event(self, nuc_number) -> None:
        """routine to check the status of the detection """
        detection_topic_name = f"/camera_{nuc_number}/fiducial_transforms"
        if not self.check_active_topic(detection_topic_name):
            self.right_bottom_frame_detect_status_label.configure(
                text="Idle", text_color="white")
            rospy.logerr(f"Detection at NUC {nuc_number} is not running..")
        else:
            self.right_bottom_frame_detect_status_label.configure(
                text="Running", text_color="yellow")
            rospy.loginfo(f"Detection at NUC {nuc_number} is running..")

    def _check_detection_rate_event(self, nuc_number) -> None:
        """ routine to check the rate of the detection """
        detection_topic_name = f"/camera_{nuc_number}/fiducial_transforms"
        if not self.check_active_topic(detection_topic_name):
            self.right_bottom_frame_detect_rate_result_label.configure(
                text="Null", text_color="white")
            rospy.logerr(f"Detection at NUC {nuc_number} is not running..")
        else:
            self.right_bottom_frame_detect_rate_result_label.configure(
                text="Running", text_color="yellow")
            rospy.loginfo(f"Detection at NUC {nuc_number} is running..")
            frequency = get_ros_topic_frequency(detection_topic_name)
            if frequency is not None:
                self.right_bottom_frame_detect_rate_result_label.configure(
                    text=f"{frequency} Hz", text_color="yellow")
                rospy.loginfo(
                    f"Frequency of {detection_topic_name}: {frequency} Hz")
            else:
                self.right_bottom_frame_detect_rate_result_label.configure(
                    text="Null", text_color="white")
                rospy.logerr("Unable to determine the frequency.")

    def _create_left_top_frame_content(self) -> None:
        """The contents of the top frame - Camera Start/Stop"""
        self.left_top_frame_label = customtkinter.CTkLabel(
            self.left_top_frame, text=f"START CAMERA - NUC {self.nuc_number}")
        self.left_top_frame_label.place(relx=0.5, rely=0.17, anchor="center")

        self.left_top_frame_start_nuc_local_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start Camera",
            command=lambda: self._start_nuc_local_cam_button_event(self.nuc_number, False))
        self.left_top_frame_start_nuc_local_cam_button.place(
            relx=0.5, rely=0.35, anchor="center")

        self.left_top_frame_start_nuc_local_cam_label_number = customtkinter.CTkLabel(
            self.left_top_frame, text="①", font=customtkinter.CTkFont(size=20), text_color=themes[COLOR_SELECT][0])
        self.left_top_frame_start_nuc_local_cam_label_number.place(
            relx=0.08, rely=0.35, anchor="center")

        self.left_top_frame_view_only_nuc_local_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="View Camera", fg_color='gray',
            command=lambda: self._view_nuc_local_cam_button_event(self.nuc_number))
        self.left_top_frame_view_only_nuc_local_cam_button.place(
            relx=0.5, rely=0.55, anchor="center")

        self.left_top_frame_view_only_nuc_local_cam_label_number = customtkinter.CTkLabel(
            self.left_top_frame, text="②", font=customtkinter.CTkFont(size=20), text_color=themes[COLOR_SELECT][0])
        self.left_top_frame_view_only_nuc_local_cam_label_number.place(
            relx=0.08, rely=0.55, anchor="center")

        self.left_top_frame_view_nuc_local_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start & View Camera", border_width=2,
            border_color=themes['red'][0],
            command=lambda: self._start_nuc_local_cam_button_event(self.nuc_number, True))
        self.left_top_frame_view_nuc_local_cam_button.place(
            relx=0.5, rely=0.80, anchor="center")

    def _create_middle_center_frame_content(self) -> None:
        """ The contents of the bottom frame - calibration parameters """
        self.middle_center_frame_label = customtkinter.CTkLabel(
            self.middle_center_frame, text=f"CALIBRATE CAMERA - NUC {self.nuc_number}")
        self.middle_center_frame_label.place(
            relx=0.5, rely=0.13, anchor="center")
        self.middle_center_frame_sq_size_label = customtkinter.CTkLabel(
            self.middle_center_frame, text="Square Size: (m)")
        self.middle_center_frame_sq_size_label.place(relx=0.1, rely=0.22)
        self.middle_center_frame_sq_size_entry = customtkinter.CTkEntry(
            master=self.middle_center_frame,
            placeholder_text=self.square_size,
            placeholder_text_color="gray"
        )
        self.middle_center_frame_sq_size_entry.place(
            relx=0.62, rely=0.22, relwidth=0.25)
        self.middle_center_frame_chessboard_label = customtkinter.CTkLabel(
            self.middle_center_frame, text="Chessboard Size: (m)")
        self.middle_center_frame_chessboard_label.place(relx=0.1, rely=0.40)
        self.middle_center_frame_chessboard_entry = customtkinter.CTkEntry(
            master=self.middle_center_frame,
            placeholder_text=self.board_size,
            placeholder_text_color="gray"
        )
        self.middle_center_frame_chessboard_entry.place(
            relx=0.62, rely=0.40, relwidth=0.25)
        self.left_button_frame_calib_update_button = customtkinter.CTkButton(
            self.middle_center_frame, text="Update",
            command=self._left_button_frame_calib_update_button_event)
        self.left_button_frame_calib_update_button.place(
            relx=0.5, rely=0.65, relwidth=0.4, anchor="center")
        self.left_button_frame_calib_update_label = customtkinter.CTkLabel(
            self.middle_center_frame,
            text_color='green',
            text='',
            font=customtkinter.CTkFont(size=20, weight="bold")
        )
        self.left_button_frame_calib_update_label.place(
            relx=0.80, rely=0.65, anchor='c')

        self.middle_center_frame_start_calib_button = customtkinter.CTkButton(
            self.middle_center_frame, text="Start Calibration",
            command=self._start_camera_calibration)
        self.middle_center_frame_start_calib_button.place(
            relx=0.5, rely=0.85, anchor="center")

    def _create_left_bottom_frame_content(self) -> None:
        """ The contents of the middle frame - Detection Start/Stop """
        self.left_bottom_frame_label = customtkinter.CTkLabel(
            self.left_bottom_frame, text=f"START DETECTION - NUC {self.nuc_number}")
        self.left_bottom_frame_label.place(relx=0.5, rely=0.17, anchor="center")
        self.left_bottom_frame_start_local_detect_button = customtkinter.CTkButton(
            self.left_bottom_frame, text="Start Detection", fg_color=themes[COLOR_SELECT][1],
            command=lambda: self._start_detection(self.nuc_number, self.detect_launch, self.uuid))
        self.left_bottom_frame_start_local_detect_button.place(
            relx=0.5, rely=0.45, anchor="center")
        self.left_bottom_frame_stop_local_detect_button = customtkinter.CTkButton(
            self.left_bottom_frame, text="Stop Detection ", fg_color='gray',
            command=lambda: self._stop_detection(self.nuc_number))
        self.left_bottom_frame_stop_local_detect_button.place(
            relx=0.5, rely=0.75, anchor="center")

    def _create_middle_bottom_frame_content(self) -> None:
        self.middle_bottom_frame_label = customtkinter.CTkLabel(
            self.middle_bottom_frame, text=f"DETECTION PARAMETERS - NUC {self.nuc_number}")
        self.middle_bottom_frame_label.place(
            relx=0.5, rely=0.13, anchor="center")
        self.middle_bottom_marker_size_label = customtkinter.CTkLabel(
            self.middle_bottom_frame, text="Marker Size: (m)")
        self.middle_bottom_marker_size_label.place(relx=0.1, rely=0.22)
        self.middle_bottom_marker_size_entry = customtkinter.CTkEntry(
            master=self.middle_bottom_frame,
            placeholder_text=self.marker_dim,
            placeholder_text_color="gray"
        )
        self.middle_bottom_marker_size_entry.place(
            relx=0.62, rely=0.22, relwidth=0.25)
        self.middle_button_dictionary_label = customtkinter.CTkLabel(
            self.middle_bottom_frame, text="Dictionary: ")
        self.middle_button_dictionary_label.place(relx=0.1, rely=0.40)
        self.middle_button_dictionary_entry = customtkinter.CTkEntry(
            master=self.middle_bottom_frame,
            placeholder_text=self.marker_dict,
            placeholder_text_color="gray"
        )
        self.middle_button_dictionary_entry.place(
            relx=0.62, rely=0.40, relwidth=0.25)
        self.middle_button_detection_update_button = customtkinter.CTkButton(
            self.middle_bottom_frame, text="Update",
            command=self._middle_button_detection_update_button_event)
        self.middle_button_detection_update_button.place(relx=0.5, rely=0.65,
                                                         relwidth=0.4, anchor="center")

    def _middle_button_detection_update_button_event(self):
        """ routine for updating detection parameters """
        marker_size_entry_var = self.middle_bottom_marker_size_entry.get()
        marker_dict_entry_var = self.middle_button_dictionary_entry.get()

        if not marker_size_entry_var and not marker_dict_entry_var:
            print('Noting Updated')
            print(f'Original marker size: {self.marker_dim}')
            print(f'Original Dictionary: {self.marker_dict}')
            rospy.logwarn('Please enter new detection parameters!')
            return
        if marker_size_entry_var:
            self.marker_dim = marker_size_entry_var
            print(f'New marker size: {self.marker_dim}')
        else:
            print(f'Original marker size: {self.marker_dim}')
        if marker_dict_entry_var:
            self.marker_dict = marker_dict_entry_var
            print(f'New Dictionary: {self.marker_dict}')
        else:
            print(f'Original Dictionary: {self.marker_dict}')

        rospy.loginfo('Detection parameters updated!')
        # self.middle_button_frame_detect_update_label.configure(text="☑", fg_color='yellow')
        print('Detection parameters updated!')

    def _start_detection(self, nuc_number, detect_launch, uuid):
        """ routine for starting dectection  """
        print('** Starting Detection **')
        print(f'Marker Size: {self.marker_dim}')
        print(f'Dictionary: {self.marker_dict}')

        start_button = self.left_bottom_frame_start_local_detect_button
        stop_button = self.left_bottom_frame_stop_local_detect_button
        try:
            if not is_node_running(f'camera_{nuc_number}/aruco_detect'):
                print('detection node is not running, now trying to start it.. ')
                detection_start(nuc_number, detect_launch, uuid,
                                self.marker_dim, self.marker_dict)
                self._check_detection_event(nuc_number)
                self._check_detection_rate_event(nuc_number)
                # self.right_bottom_frame_detect_status_label.configure(text="Running", text_color="yellow")
                if is_node_running(f'camera_{nuc_number}/aruco_detect'):
                    start_button.configure(fg_color=themes['red'])
                    stop_button.configure(fg_color=themes['green'])
            else:
                rospy.logwarn(
                    f'NUC {nuc_number} Detection is already running..')
        except Exception as e:
            print(f'Error: {e}')

    def _stop_detection(self, nuc_number):
        """ routine for stopping dectection """
        start_button = self.left_bottom_frame_start_local_detect_button
        stop_button = self.left_bottom_frame_stop_local_detect_button
        try:
            if is_node_running(f'/camera_{nuc_number}/aruco_detect'):
                print('detection node is running, now trying to stop it.. ')
                detection_stop(nuc_number)
                if not is_node_running(f'camera_{nuc_number}/aruco_detect'):
                    self._check_detection_event(nuc_number)
                    self._check_detection_rate_event(nuc_number)
                    # self.right_bottom_frame_detect_status_label.configure(text="Idle", text_color="white")
                    start_button.configure(fg_color=themes[COLOR_SELECT][0])
                    stop_button.configure(fg_color='gray')
            else:
                rospy.logerr(f'NUC {nuc_number} Detection is not running..')
        except Exception as e:
            print(f'Error: {e}')

    def check_active_topic(self, topic_name):
        """Checks whether a topic is currently running/active or not.. """
        all_topics = rospy.get_published_topics()
        if topic_name in [topic[0] for topic in all_topics]:
            return True
        else:
            return False

    def _view_nuc_local_cam_button_event(self, view_nuc_machine) -> None:
        """This function is used to view camera output for a given camera """
        node_name = f"/camera_{view_nuc_machine}"
        view_node_name = f"/camera_{view_nuc_machine}_view"
        if not is_node_running(node_name):
            rospy.logerr(f"Camera at NUC {view_nuc_machine} is not running..")
        else:
            if not is_node_running(view_node_name):
                print('Now displaying camera output..')
                cam_view_args = [f'{self.view_launch}',
                                 f'camera_name:=camera_{view_nuc_machine}']
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cam_view_args)[0],
                                   cam_view_args[1:])]
                nuc_cam_view = roslaunch.parent.ROSLaunchParent(
                    self.uuid, roslaunch_file)
                nuc_cam_view.start()
                rospy.loginfo(
                    f"Camera View at NUC {view_nuc_machine} started successfully.!!")
                self.left_top_frame_view_only_nuc_local_cam_button.configure(
                    text='Stop Camera View', fg_color=themes['red'])

            else:
                rospy.loginfo("Now Stopping Camera Camera View.!!")
                try:
                    if is_node_running(view_node_name):
                        kill_ros_node(view_node_name)
                        rospy.loginfo(
                            f'NUC {view_nuc_machine} Camera View stopped successfully!')
                        self.left_top_frame_view_only_nuc_local_cam_button.configure(
                            text='View Camera', fg_color=themes[COLOR_SELECT][0])
                except (e):
                    rospy.logerr(f"Error Stopping Camera View.!!, {e}")

    def _start_nuc_local_cam_button_event(self, nuc_machine, show_camera) -> None:
        """This function is used to start or stop the camera node based on its current state."""
        # If camera is not running, start it
        camera_topic_name = f"/camera_{nuc_machine}/image_raw"
        if not self.check_active_topic(camera_topic_name):
            camera_launch_args = [
                f"{self.local_nuc_launch}", f"launch_nuc:=camera_{nuc_machine}"]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(camera_launch_args)[0],
                               camera_launch_args[1:])]
            nuc_cam_driver = roslaunch.parent.ROSLaunchParent(
                self.uuid, roslaunch_file)
            nuc_cam_driver.start()
            self.running_processes[f'camera_{nuc_machine}_driver'] = nuc_cam_driver
            rospy.sleep(1)
            self._update_camera_specs(nuc_machine)
            self._check_camera_fps_event(nuc_machine)

            # Update button text to indicate that the camera can be stopped
            rospy.loginfo(f'NUC {nuc_machine} Camera started successfully!')
            if show_camera:
                print('Now displaying camera output..')
                cam_view_args = [f'{self.view_launch}',
                                 f'camera_name:=camera_{nuc_machine}']
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cam_view_args)[0],
                                   cam_view_args[1:])]
                nuc_cam_view = roslaunch.parent.ROSLaunchParent(
                    self.uuid, roslaunch_file)

                nuc_cam_view.start()
                self.running_processes[f'camera_{nuc_machine}_view'] = nuc_cam_view
                # Update button text to indicate that the camera can be stopped
                self.left_top_frame_view_nuc_local_cam_button.configure(text="Stop View & Camera ",
                                                                        fg_color=themes["red"])
                rospy.loginfo(
                    f'NUC {nuc_machine} Camera view started successfully!')
            else:
                self.left_top_frame_start_nuc_local_cam_button.configure(text="Stop Camera",
                                                                         fg_color=themes["red"])
                self.left_top_frame_view_only_nuc_local_cam_button.configure(state='normal',
                                                                             fg_color=themes['green'])
        # If camera is running, stop it
        else:
            if show_camera:
                try:
                    self.running_processes[f'camera_{nuc_machine}_view'].shutdown(
                    )
                    self.running_processes.pop(
                        f'camera_{nuc_machine}_view', None)
                    rospy.loginfo(
                        f'NUC {nuc_machine} Camera view stopped successfully!')
                except roslaunch.RLException as excep_view:
                    rospy.logerr(
                        f'Error stopping nuc{nuc_machine} camera view: {str(excep_view)}')
                finally:
                    self.left_top_frame_view_nuc_local_cam_button.configure(text="Start & View Camera",
                                                                            fg_color=themes[COLOR_SELECT][0])

            try:
                self.running_processes[f'camera_{nuc_machine}_driver'].shutdown(
                )
                self.running_processes.pop(
                    f'camera_{nuc_machine}_driver', None)
                self._update_camera_specs(nuc_machine)
                self._check_camera_fps_event(nuc_machine)
                rospy.loginfo(
                    f'NUC {nuc_machine} Camera stopped successfully!')
            except roslaunch.RLException as excep_camera:
                rospy.logerr(
                    f'Error stopping nuc{nuc_machine} camera driver: {str(excep_camera)}')
            finally:
                # Update button text to indicate that the camera can be started
                self.left_top_frame_start_nuc_local_cam_button.configure(text="Start Camera",
                                                                         fg_color=themes[COLOR_SELECT][0])
                self.left_top_frame_view_only_nuc_local_cam_button.configure(
                    fg_color='gray')

    def _start_camera_calibration(self):
        print('** Starting Camera Calibration **')
        print(f'Board Size: {self.board_size}')
        print(f'Square Size: {self.square_size}')
        self._start_nuc_local_cam_button_event(
            self.nuc_number, show_camera=False)
        cmd = ['rosrun', 'camera_calibration', 'cameracalibrator.py',
               '--size', self.board_size, '--square', self.square_size, '--k-coefficients=2',
               '--fix-principal-point', 'i', '--fix-aspect-ratio',
               f'image:=/camera_{self.nuc_number}/image_raw', f'camera:=/camera_{self.nuc_number}']

        # Execute the command
        subprocess.call(cmd)
        self._start_nuc_local_cam_button_event(
            self.nuc_number, show_camera=False)

    def _left_button_frame_calib_update_button_event(self):
        """ routine for updating calibration parameters """
        chessboard_entry = self.middle_center_frame_chessboard_entry.get()
        sq_size_entry = self.middle_center_frame_sq_size_entry.get()
        if not chessboard_entry and not sq_size_entry:
            print('Nothing updated')
            print(f'Original square size: {self.square_size}')
            print(f'Original board size: {self.board_size}')
            rospy.logwarn('Please enter new calibration parameters!')
            return

        if chessboard_entry:
            self.board_size = chessboard_entry
            print(f'Updated board size: {self.board_size}')
        else:
            print(f'Original board size: {self.board_size}')

        if sq_size_entry:
            self.square_size = sq_size_entry
            print(f'Updated square size: {self.square_size}')
        else:
            print(f'Original square size: {self.square_size}')

        rospy.loginfo('Checkerboard parameters updated successfully')
        self.left_button_frame_calib_update_label.configure(
            text="☑", fg_color='yellow')
        print('Chessboard Parameters Updated!')

    def on_slider_change(self, value):
        """ Function to change the exposure of the camera """
        # gain_value = value
        # try:
        #     self.set_gain(gain_value)
        #     print(f'Exposure changed to {gain_value} successfully!')
        # except Exception as e:
        #     print(f'Error in the service calll: {e}')
        # self.right_bottom_frame_cam_exposure_result_label.
        # self._set_camera_exposure(value)
        pass
    

if __name__ == "__main__":
    root = ClientGUI()
    root.mainloop()
    