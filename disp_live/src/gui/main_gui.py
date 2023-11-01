#!/usr/bin/env python3
""" backend definitions for the gui"""

import subprocess
import tkinter as tk
import customtkinter
import rospy
import rospkg
import roslaunch
import time
from _backend_ import *
from _backend_ import _check_ros_status_function



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
        rospy.init_node("main_gui", anonymous=False)
        self.package = 'gige_cam_driver'
        # ********************************************************************************
        # Path management for Launch files
        # ********************************************************************************
        self.launch_path = rospkg.RosPack().get_path('gige_cam_driver') + '/launch/'
        self.detect_launch_path = rospkg.RosPack().get_path('aruco_detect') + '/launch/'
        self.remote_nuc_launch = f'{self.launch_path}remote_nuc.launch'
        self.remote_detect_launch = f'{self.detect_launch_path}remote_aruco.launch'

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(self.uuid)
        self.left_width = 0.40

        self.nuc_number = '2'
        self.title("Displacements Dashboard")
        self.geometry("1000x600")
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.destroy_routine)
        # Create a BooleanVar to use as the variable for the checkbox
        self.view_camera = tk.BooleanVar()
        self.view_camera.set(False)  # Set the initial value to False
        self.maker_size = "0.02"
        self.aruco_dict = "0"
        self.running_processes = {}
        self.left_frame = None
        self.left_top_frame = None
        self.left_top_frame_label = None
        self.left_top_frame_button = None
        self.left_bottom_frame = None
        self.left_bottom_frame_label = None
        self.left_bottom_frame_start_calib_button = customtkinter.CTkButton(
            master=self.left_bottom_frame)
        self.left_bottom_frame_dict_label = None
        self.left_bottom_frame_dict_entry = None
        self.right_top_frame_system_label = None
        self.right_top_frame_label = None
        self.left_top_frame_view_cam_checkbox = None
        self.left_bottom_frame_marker_size_label = None
        self.left_bottom_frame_marker_size_entry = None
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
        self._create_left_middle_frame()
        self._create_left_bottom_frame()
    def _create_left_top_frame(self) -> None:
        """_summary_"""
        self.left_top_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_top_frame.place(relx=0.1, rely=0.04, relwidth=0.8, relheight=0.30)
        self._create_left_top_frame_content()
    def _create_left_top_frame_content(self) -> None:
        """_summary_"""
        self.left_top_frame_label = customtkinter.CTkLabel(
            self.left_top_frame, text="START REMOTE CAMERA")
        self.left_top_frame_label.place(relx=0.5, rely=0.15, anchor="center")

        self.left_top_frame_start_nuc1_remote_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start Camera - NUC 1",
            command=lambda: self._start_nuc_remote_cam_button_event_updated(1))
        self.left_top_frame_start_nuc1_remote_cam_button.place(relx=0.27, rely=0.35, anchor="center")

        self.left_top_frame_stop_nuc1_remote_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="Stop Camera - NUC 1",
            command=lambda: self._stop_nuc_remote_cam_button_event_updated(1), state="disabled")
        self.left_top_frame_stop_nuc1_remote_cam_button.place(relx=0.73, rely=0.35, anchor="center")


        self.left_top_frame_start_nuc2_remote_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start Camera - NUC 2",
            command=lambda: self._start_nuc_remote_cam_button_event_updated(2))
        self.left_top_frame_start_nuc2_remote_cam_button.place(relx=0.27, rely=0.56, anchor="center")
        
        self.left_top_frame_stop_nuc2_remote_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="Stop Camera - NUC 2",
            command=lambda: self._stop_nuc_remote_cam_button_event_updated(2), state="disabled")
        self.left_top_frame_stop_nuc2_remote_cam_button.place(relx=0.73, rely=0.56, anchor="center")

        self.left_top_frame_start_nuc3_remote_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start Camera - NUC 3",
            command=lambda: self._start_nuc_remote_cam_button_event_updated(3))
        self.left_top_frame_start_nuc3_remote_cam_button.place(relx=0.27, rely=0.77, anchor="center")

        self.left_top_frame_stop_nuc3_remote_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="Stop Camera - NUC 3",
            command=lambda: self._stop_nuc_remote_cam_button_event_updated(3), state="disabled")
        self.left_top_frame_stop_nuc3_remote_cam_button.place(relx=0.73, rely=0.77, anchor="center")


    def _start_nuc_remote_cam_button_event_updated(self, _nuc_number) -> None:
        remote_cam_start_updated(machine_num = _nuc_number,
                                 remote_nuc_launch = self.remote_nuc_launch,
                                 ros_uuid = self.uuid,
                                 start_btn = self.start_button_address(_nuc_number),
                                 stop_btn  = self.stop_button_address(_nuc_number))
        _check_ros_status_function(self.right_top_frame_ros_status_result_label)

        
    def _stop_nuc_remote_cam_button_event_updated(self, _nuc_number) -> None:
        remote_cam_stop_updated(machine_num = _nuc_number,
                                 start_btn = self.start_button_address(_nuc_number),
                                 stop_btn  = self.stop_button_address(_nuc_number))
        _check_ros_status_function(self.right_top_frame_ros_status_result_label)

    def _start_nuc_remote_detec_button_event(self, _nuc_number) -> None:
        remote_detect_start(machine_num = _nuc_number,
                            remote_detect_launch_file = self.remote_detect_launch,
                            ros_uuid = self.uuid,
                            start_detect_btn = self.start_detect_button_address(_nuc_number),
                            stop_detect_btn = self.stop_detect_button_address(_nuc_number))

    def _stop_nuc_remote_detect_button_event(self, _nuc_number) -> None:
        remote_detect_stop(machine_num = _nuc_number)
                           # start_btn = self.start_button_address(_nuc_number),
                           # stop_btn  = self.stop_button_address(_nuc_number))
    def start_detect_button_address(self, _nuc_number):
        if _nuc_number == 1:
            return self.right_middle_frame_start_nuc1_detect_button
        elif _nuc_number == 2:
            return self.right_middle_frame_start_nuc2_detect_button
        elif _nuc_number == 3:
            return self.right_middle_frame_start_nuc3_detect_button
    def stop_detect_button_address(self, _nuc_number):
        if _nuc_number == 1:
            return self.right_middle_frame_stop_nuc1_detect_button
        elif _nuc_number == 2:
            return self.right_middle_frame_stop_nuc2_detect_button
        elif _nuc_number == 3:
            return self.right_middle_frame_stop_nuc3_detect_button
        
    def start_button_address(self, _nuc_number):
        if _nuc_number == 1:
            return self.left_top_frame_start_nuc1_remote_cam_button
        elif _nuc_number == 2:
            return self.left_top_frame_start_nuc2_remote_cam_button
        elif _nuc_number == 3:
            return self.left_top_frame_start_nuc3_remote_cam_button
    def stop_button_address(self, _nuc_number):
        if _nuc_number == 1:
            return self.left_top_frame_stop_nuc1_remote_cam_button
        elif _nuc_number == 2:
            return self.left_top_frame_stop_nuc2_remote_cam_button
        elif _nuc_number == 3:
            return self.left_top_frame_stop_nuc3_remote_cam_button
    
    def _create_left_middle_frame(self) -> None:
        """_summary_
        """
        self.left_middle_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_middle_frame.place(relx=0.1, rely=0.38, relwidth=0.8, relheight=0.14)
        self._create_left_middle_frame_content()
    def _create_left_middle_frame_content(self) -> None:
        """_summary_
        """
        self.left_middle_frame_label = customtkinter.CTkLabel(
        self.left_middle_frame, text="START REMOTE CAMERAS")
        self.left_middle_frame_label.place(relx=0.5, rely=0.25, anchor="center")

        self.left_middle_frame_start_all_cams_button = customtkinter.CTkButton(
            self.left_middle_frame, text="Start All NUC Cameras",
            command=self._start_all_nuc_remote_cams_button_event
            )
        self.left_middle_frame_start_all_cams_button.place(relx=0.27, rely=0.6, anchor="center")
        
        self.left_middle_frame_stop_all_cams_button = customtkinter.CTkButton(
            self.left_middle_frame, text="Stop All NUC Cameras",
            command=self._stop_all_nuc_remote_cams_button_event, state="disabled"
            )
        self.left_middle_frame_stop_all_cams_button.place(relx=0.73, rely=0.6, anchor="center")
    def _start_all_nuc_remote_cams_button_event(self) -> None:
        """Starting all the nuc cameras remotely """
        # nuc1_status, nuc2_status, nuc3_status = return_nuc_status()
        # if not (nuc1_status and nuc2_status and nuc3_status):
        self._start_nuc_remote_cam_button_event_updated(1)
        self._start_nuc_remote_cam_button_event_updated(2)
        self._start_nuc_remote_cam_button_event_updated(3)
        time.sleep(1)
        self.left_middle_frame_start_all_cams_button.configure(fg_color=themes['green'])
        self.left_middle_frame_stop_all_cams_button.configure(fg_color=themes['red'])
        self.left_middle_frame_stop_all_cams_button.configure(state='normal')
        print('All cameras started successfully!')
    def _stop_all_nuc_remote_cams_button_event(self) -> None:
        self._stop_nuc_remote_cam_button_event_updated(1)
        self._stop_nuc_remote_cam_button_event_updated(2)
        self._stop_nuc_remote_cam_button_event_updated(3)
        self.left_middle_frame_start_all_cams_button.configure(fg_color=themes[COLOR_SELECT][0])
        self.left_middle_frame_stop_all_cams_button.configure(fg_color=themes[COLOR_SELECT][0])
        self.left_middle_frame_stop_all_cams_button.configure(state='disabled')
    def _create_left_bottom_frame(self) -> None:
        """_summary_
        """
        self.left_bottom_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_bottom_frame.place(relx=0.1, rely=0.56, relwidth=0.8, relheight=0.32)
        self._create_left_bottom_frame_content()
    def _create_left_bottom_frame_content(self) -> None:
        """_summary_
        """
        self.left_bottom_frame_label = customtkinter.CTkLabel(
        self.left_bottom_frame, text="DETECTION PARAMETERS")
        self.left_bottom_frame_label.place(relx=0.5, rely=0.13, anchor="center")

        self.left_bottom_frame_dict_label = customtkinter.CTkLabel(
            self.left_bottom_frame, text="Dictionary")
        self.left_bottom_frame_dict_label.place(relx=0.1, rely=0.22)

        self.left_bottom_frame_dict_entry = customtkinter.CTkEntry(
            master=self.left_bottom_frame,
            placeholder_text=self.aruco_dict,
            placeholder_text_color="gray",
            # state="disabled"
        )
        self.left_bottom_frame_dict_entry.place(relx=0.62, rely=0.22, relwidth=0.25)

        self.left_bottom_frame_marker_size_label = customtkinter.CTkLabel(
            self.left_bottom_frame, text="Marker Size: (m)")
        self.left_bottom_frame_marker_size_label.place(relx=0.1, rely=0.40)

        self.left_bottom_frame_marker_size_entry = customtkinter.CTkEntry(
            master=self.left_bottom_frame,
            placeholder_text=self.maker_size,
            placeholder_text_color="gray"
        )
        self.left_bottom_frame_marker_size_entry.place(relx=0.62, rely=0.40, relwidth=0.25)

        self.left_button_frame_marker_update_button = customtkinter.CTkButton(
            self.left_bottom_frame, text="Update",
            command=self._left_button_frame_marker_update_button_event)
        self.left_button_frame_marker_update_button.place(relx=0.5, rely=0.65,
                                                         relwidth=0.4, anchor="center")

        self.left_button_frame_maker_update_label = customtkinter.CTkLabel(
            self.left_bottom_frame,
            text_color='green',
            text='',
            font=customtkinter.CTkFont(size=20, weight="bold")
            )
        self.left_button_frame_maker_update_label.place(relx=0.80, rely=0.65, anchor='c')
    def _left_button_frame_marker_update_button_event(self):
        marker_entry = self.left_bottom_frame_marker_size_entry.get()
        sq_size_entry = self.left_bottom_frame_dict_entry.get()

        if not marker_entry and not sq_size_entry:
            print('Nothing updated')
            print(f'Original Dictionary: {self.aruco_dict}')
            print(f'Original Marker Size: {self.maker_size}')
            rospy.logwarn('Please enter new marker parameters!')
            return

        if marker_entry:
            self.maker_size = marker_entry
            print(f'Updated Marker Size: {self.maker_size}')
        else:
            print(f'Original Marker Size: {self.maker_size}')

        if sq_size_entry:
            self.aruco_dict = sq_size_entry
            print(f'Updated Dictionary: {self.aruco_dict}')
        else:
            print(f'Original Dictionary: {self.aruco_dict}')

        rospy.loginfo('Marker parameters updated successfully')
        self.left_button_frame_maker_update_label.configure(text="☑", fg_color='yellow')
        print('Marker Parameters Updated!')
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
        self.right_middle_frame_detect_label = customtkinter.CTkLabel(
            self.right_middle_frame, text="START MARKER DETECTION", text_color="white")
        self.right_middle_frame_detect_label.place(relx=0.5, rely=0.15, anchor="center")
            
        self.right_middle_frame_start_nuc1_detect_button = customtkinter.CTkButton(
            self.right_middle_frame, text="Start NUC 1 Detection", fg_color=themes[COLOR_SELECT][1],
            command=lambda: self._start_nuc_remote_detec_button_event(1))
        self.right_middle_frame_start_nuc1_detect_button.place(relx=0.21, rely=0.35, anchor="center")

        self.right_middle_frame_start_nuc2_detect_button = customtkinter.CTkButton(
            self.right_middle_frame, text="Start NUC 2 Detection", fg_color=themes[COLOR_SELECT][1],
            command=lambda: self._start_nuc_remote_detec_button_event(2))
        self.right_middle_frame_start_nuc2_detect_button.place(relx=0.48, rely=0.35, anchor="center")
        
        self.right_middle_frame_start_nuc3_detect_button = customtkinter.CTkButton(
            self.right_middle_frame, text="Start NUC 3 Detection", fg_color=themes[COLOR_SELECT][1],
            command=lambda: self._start_nuc_remote_detec_button_event(3))
        self.right_middle_frame_start_nuc3_detect_button.place(relx=0.75, rely=0.35, anchor="center")

        self.right_middle_frame_stop_nuc1_detect_button = customtkinter.CTkButton(
            self.right_middle_frame, text="Stop NUC 1 Detection", fg_color=themes[COLOR_SELECT][1],
            command=lambda: self._stop_nuc_remote_detect_button_event(1), state="disabled")
        self.right_middle_frame_stop_nuc1_detect_button.place(relx=0.21, rely=0.55, anchor="center")

        self.right_middle_frame_stop_nuc2_detect_button = customtkinter.CTkButton(
            self.right_middle_frame, text="Stop NUC 2 Detection", fg_color=themes[COLOR_SELECT][1],
            command=lambda: self._stop_nuc_remote_detect_button_event(2), state="disabled")
        self.right_middle_frame_stop_nuc2_detect_button.place(relx=0.48, rely=0.55, anchor="center")

        self.right_middle_frame_stop_nuc3_detect_button = customtkinter.CTkButton(
            self.right_middle_frame, text="Stop NUC 3 Detection", fg_color=themes[COLOR_SELECT][1],
            command=lambda: self._stop_nuc_remote_detect_button_event(3), state="disabled")
        self.right_middle_frame_stop_nuc3_detect_button.place(relx=0.75, rely=0.55, anchor="center")
        
        self.right_middle_frame_start_all_detect_button = customtkinter.CTkButton(
            self.right_middle_frame, text="Start All NUCs Detection", fg_color=themes[COLOR_SELECT][1],
            command=self._start_all_nuc_remote_cams_button_event)
        self.right_middle_frame_start_all_detect_button.place(relx=0.282, rely=0.80, anchor="center", relwidth = 0.4)

        self.right_middle_frame_stop_all_detect_button = customtkinter.CTkButton(
            self.right_middle_frame, text="Stop All NUCs Detection", fg_color=themes[COLOR_SELECT][1],
            command=self._start_all_nuc_remote_cams_button_event, state="disabled")
        self.right_middle_frame_stop_all_detect_button.place(relx=0.7, rely=0.80, anchor="center", relwidth = 0.4)        
        
        
        
        
    def _create_right_lower_frame_content(self) -> None:
        pass
    
if __name__ == "__main__":
    root = ClientGUI()
    root.mainloop()
