#!/usr/bin/env python3
""" backend definitions for the gui"""

import subprocess
import tkinter as tk
import customtkinter
import rospy
import rospkg
import roslaunch
from _backend_ import is_node_running, kill_ros_node, detection_start, detection_stop

themes = {'blue': ("#3B8ED0", "#1F6AA5"),
          'green': ("#2CC985", "#2FA572"),
          'dark-blue': ("#3a7ebf", "#1f538d"),
          'red':("#fa5f5a", "#ba3732")
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
        self.nuc_number = '3'
        rospy.init_node(f"nuc{self.nuc_number}_gui", anonymous=False)
        self.package = 'gige_cam_driver'
        # ********************************************************************************
        # Path management for Launch files
        # ********************************************************************************
        self.launch_path = rospkg.RosPack().get_path('gige_cam_driver') + '/launch/'
        self.csv_folder_path = rospkg.RosPack().get_path('gige_cam_driver') + '/csvfiles/'
        self.detect_launch_path = rospkg.RosPack().get_path('aruco_detect') + '/launch/'

        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        # roslaunch.configure_logging(self.uuid)
        # self.cam_launch = f"{self.launch_path}cam.launch"
        self.local_nuc_launch = f'{self.launch_path}local_nuc.launch'
        self.view_launch = f"{self.launch_path}viewcam.launch"
        self.calib_launch = f"{self.launch_path}calib.launch"
        self.detect_launch = f"{self.detect_launch_path}local_detect.launch"

        
        self.title(f"NUC {self.nuc_number} Dashboard")
        self.geometry("1000x600")
        self.resizable(False, False)
        self.protocol("WM_DELETE_WINDOW", self.destroy_routine)
        # Create a BooleanVar to use as the variable for the checkbox
        self.view_camera = tk.BooleanVar()
        self.view_camera.set(False)  # Set the initial value to False
        self.nuc1_camera = False
        self.nuc2_camera = False
        self.nuc3_camera = False
        self.board_size = "9x8"
        self.square_size = "0.025"  # in meters
        self.var_dictionary = tk.StringVar(self, "0")  # dict 5x5 (1000)
        self.running_processes = {}
        self.left_frame = None
        self.left_top_frame = None
        self.left_top_frame_label = None
        self.left_top_frame_button = None
        self.left_bottom_frame = None
        self.left_bottom_frame_label = None
        self.left_bottom_frame_start_calib_button = customtkinter.CTkButton(
            master=self.left_bottom_frame)
        self.left_bottom_frame_sq_size_label = None
        self.left_bottom_frame_sq_size_entry = None
        self.right_top_frame_system_label = None
        self.right_top_frame_label = None
        self.left_top_frame_view_cam_checkbox = None
        self.left_bottom_frame_chessboard_label = None
        self.left_bottom_frame_chessboard_entry = None
        self.left_button_frame_calib_update_button = customtkinter.CTkButton(
            master=self.left_bottom_frame)
        self.right_frame = None
        self.right_top_frame = None
        self.right_top_frame_ros_status_label = None
        self.right_top_frame_ros_status_result_label = None
        self.right_top_frame_camera_label = None
        self.right_top_frame_camera_result_label = None
        self.right_bottom_frame = None
        self.left_middle_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_middle_frame_label = customtkinter.CTkLabel(self.left_middle_frame)
        self.left_button_frame_calib_update_label = customtkinter.CTkLabel(self.left_bottom_frame)
        self.left_middle_frame_start_local_detect_button = customtkinter.CTkButton(
            self.left_middle_frame)
        self.left_middle_frame_start_nuc2_cam_button = customtkinter.CTkButton(
            self.left_middle_frame)
        self.left_middle_frame_start_nuc3_cam_button = customtkinter.CTkButton(
            self.left_middle_frame)
        self._create_widgets()

    def destroy_routine(self) -> None:
        """_summary_"""
        self.destroy()
        self.quit()

    def _create_widgets(self) -> None:
        """Starting point of the GUI"""
        self._create_left_frame()
        self._create_right_frame()

    def _create_left_frame(self) -> None:
        """ routine to create the whole left panel of GUI """
        self.left_frame = tk.Frame(self, bg=themes[COLOR_SELECT][1])
        self.left_frame.place(relx=0, rely=0, relwidth=0.25, relheight=1)
        self._create_left_top_frame()
        self._create_left_middle_frame()
        # self._create_left_bottom_frame()

    def _create_left_top_frame(self) -> None:
        """ routine to create the top frame of the left panel of GUI"""
        self.left_top_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_top_frame.place(relx=0.1, rely=0.04, relwidth=0.8, relheight=0.30)
        self._create_left_top_frame_content()
    def _create_left_middle_frame(self) -> None:
        """ routine to create the middle frame of the left panel of GUI """
        self.left_middle_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_middle_frame.place(relx=0.1, rely=0.37, relwidth=0.8, relheight=0.2)
        self._create_left_middle_frame_content()
    def _create_left_bottom_frame(self) -> None:
        """_summary_
        """
        self.left_bottom_frame = customtkinter.CTkFrame(self.left_frame)
        self.left_bottom_frame.place(relx=0.1, rely=0.28, relwidth=0.8, relheight=0.32)
        self._create_left_bottom_frame_content()

    def _create_left_top_frame_content(self) -> None:
        """The contents of the top frame - Camera Start/Stop"""
        self.left_top_frame_label = customtkinter.CTkLabel(
            self.left_top_frame, text=f"START CAMERA - NUC {self.nuc_number}")
        self.left_top_frame_label.place(relx=0.5, rely=0.17, anchor="center")

        self.left_top_frame_start_nuc_local_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start Camera",
            command=lambda: self._start_nuc_local_cam_button_event(self.nuc_number, False))
        self.left_top_frame_start_nuc_local_cam_button.place(relx=0.5, rely=0.35, anchor="center")
        
        self.left_top_frame_start_nuc_local_cam_label_number = customtkinter.CTkLabel(
            self.left_top_frame, text="①", font=customtkinter.CTkFont(size=20), text_color=themes[COLOR_SELECT][0])
        self.left_top_frame_start_nuc_local_cam_label_number.place(relx=0.08, rely=0.35, anchor="center")


        self.left_top_frame_view_only_nuc_local_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="View Camera", fg_color='gray',
            command=lambda: self._view_nuc_local_cam_button_event(self.nuc_number))
        self.left_top_frame_view_only_nuc_local_cam_button.place(relx=0.5, rely=0.55, anchor="center")
        
        self.left_top_frame_view_only_nuc_local_cam_label_number = customtkinter.CTkLabel(
            self.left_top_frame, text="②", font=customtkinter.CTkFont(size=20), text_color=themes[COLOR_SELECT][0])
        self.left_top_frame_view_only_nuc_local_cam_label_number.place(relx=0.08, rely=0.55, anchor="center")
        

        self.left_top_frame_view_nuc_local_cam_button = customtkinter.CTkButton(
            self.left_top_frame, text="Start & View Camera", border_width=2,
            border_color=themes['red'][0], 
            command=lambda: self._start_nuc_local_cam_button_event(self.nuc_number, True))
        self.left_top_frame_view_nuc_local_cam_button.place(relx=0.5, rely=0.80, anchor="center")
    def _create_left_middle_frame_content(self) -> None:
        """ The contents of the middle frame - Detection Start/Stop """
        self.left_middle_frame_label = customtkinter.CTkLabel(
        self.left_middle_frame, text=f"START DETECTION - NUC {self.nuc_number}")
        self.left_middle_frame_label.place(relx=0.5, rely=0.17, anchor="center")
        self.left_middle_frame_start_local_detect_button = customtkinter.CTkButton(
            self.left_middle_frame, text="Start Detection", fg_color=themes[COLOR_SELECT][1],
            command=lambda: detection_start(self.nuc_number, self.detect_launch, self.uuid, self.left_middle_frame_start_local_detect_button, self.left_middle_frame_stop_local_detect_button))
        self.left_middle_frame_start_local_detect_button.place(relx=0.5, rely=0.45, anchor="center")
        self.left_middle_frame_stop_local_detect_button = customtkinter.CTkButton(
            self.left_middle_frame, text="Stop Detection ", fg_color='gray',
            command=lambda: detection_stop(self.nuc_number, self.left_middle_frame_start_local_detect_button, self.left_middle_frame_stop_local_detect_button))
        self.left_middle_frame_stop_local_detect_button.place(relx=0.5, rely=0.75, anchor="center")
    def _create_left_bottom_frame_content(self) -> None:
        """ The contents of the bottom frame - calibration parameters """
        self.left_bottom_frame_label = customtkinter.CTkLabel(
        self.left_bottom_frame, text=f"CALIBRATE CAMERA - NUC {self.nuc_number}")
        self.left_bottom_frame_label.place(relx=0.5, rely=0.13, anchor="center")
        self.left_bottom_frame_sq_size_label = customtkinter.CTkLabel(
            self.left_bottom_frame, text="Square Size: (m)")
        self.left_bottom_frame_sq_size_label.place(relx=0.1, rely=0.22)
        self.left_bottom_frame_sq_size_entry = customtkinter.CTkEntry(
            master=self.left_bottom_frame,
            placeholder_text=self.square_size,
            placeholder_text_color="gray"
        )
        self.left_bottom_frame_sq_size_entry.place(relx=0.62, rely=0.22, relwidth=0.25)
        self.left_bottom_frame_chessboard_label = customtkinter.CTkLabel(
            self.left_bottom_frame, text="Chessboard Size: (m)")
        self.left_bottom_frame_chessboard_label.place(relx=0.1, rely=0.40)
        self.left_bottom_frame_chessboard_entry = customtkinter.CTkEntry(
            master=self.left_bottom_frame,
            placeholder_text=self.board_size,
            placeholder_text_color="gray"
        )
        self.left_bottom_frame_chessboard_entry.place(relx=0.62, rely=0.40, relwidth=0.25)
        self.left_button_frame_calib_update_button = customtkinter.CTkButton(
            self.left_bottom_frame, text="Update",
            command=self._left_button_frame_calib_update_button_event)
        self.left_button_frame_calib_update_button.place(relx=0.5, rely=0.65,
                                                         relwidth=0.4, anchor="center")
        self.left_button_frame_calib_update_label = customtkinter.CTkLabel(
            self.left_bottom_frame,
            text_color='green',
            text='',
            font=customtkinter.CTkFont(size=20, weight="bold")
            )
        self.left_button_frame_calib_update_label.place(relx=0.80, rely=0.65, anchor='c')

        self.left_bottom_frame_start_calib_button = customtkinter.CTkButton(
            self.left_bottom_frame,text="Start Calibration",
            command=self._start_camera_calibration)
        self.left_bottom_frame_start_calib_button.place(relx=0.5, rely=0.85, anchor="center")
    
    def _start_nuc_remote_cam_button_event(self, camera_number) -> None:
        print(f"Starting Camera {camera_number} from NUC {self.nuc_number}...")
    def check_active_topic(self, topic_name):
        """Checks whether a topic is currently running/active or not.. """
        all_topics = rospy.get_published_topics()
        if topic_name in [topic[0] for topic in all_topics]:
            return True
        else:
            return False
    def _view_nuc_local_cam_button_event(self, view_nuc_machine) -> None:
        """This function is used to view camera output for a given camera """
        node_name = f"/nuc{view_nuc_machine}"
        view_node_name = f"/nuc{view_nuc_machine}_view"
        if not is_node_running(node_name):
            rospy.logerr(f"Camera at NUC {view_nuc_machine} is not running..")
        else:
            if not is_node_running(view_node_name):
                print('Now displaying camera output..')
                cam_view_args = [f'{self.view_launch}',
                                 f'camera_name:=nuc{view_nuc_machine}']
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cam_view_args)[0],
                                   cam_view_args[1:])]
                nuc_cam_view = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
                nuc_cam_view.start()
                rospy.loginfo(f"Camera View at NUC {view_nuc_machine} started successfully.!!")
                self.left_top_frame_view_only_nuc_local_cam_button.configure(text='Stop Camera View', fg_color=themes['red'])
                
            else:
                rospy.loginfo("Now Stopping Camera Camera View.!!")
                try:
                    if is_node_running(view_node_name):
                        kill_ros_node(view_node_name)
                        rospy.loginfo(f'NUC {view_nuc_machine} Camera View stopped successfully!')
                        self.left_top_frame_view_only_nuc_local_cam_button.configure(text='View Camera', fg_color=themes[COLOR_SELECT][0])
                except (e):
                    rospy.logerr(f"Error Stopping Camera View.!!, {e}")
    def _start_nuc_local_cam_button_event(self, nuc_machine, show_camera) -> None:
        """This function is used to start or stop the camera node based on its current state."""
        # If camera is not running, start it
        camera_topic_name = f"/nuc{nuc_machine}/image_raw"
        if not self.check_active_topic(camera_topic_name):
            camera_launch_args = [f"{self.local_nuc_launch}", f"launch_nuc:=nuc{nuc_machine}"]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(camera_launch_args)[0],
                               camera_launch_args[1:])]
            nuc_cam_driver = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
            nuc_cam_driver.start()
            self.running_processes[f'nuc{nuc_machine}_driver'] = nuc_cam_driver
            # Update button text to indicate that the camera can be stopped
            rospy.loginfo(f'NUC {nuc_machine} Camera started successfully!')
            if show_camera:
                print('Now displaying camera output..')
                cam_view_args = [f'{self.view_launch}',
                                 f'camera_name:=nuc{nuc_machine}']
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cam_view_args)[0],
                                   cam_view_args[1:])]
                nuc_cam_view = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)

                nuc_cam_view.start()
                self.running_processes[f'nuc{nuc_machine}_view'] = nuc_cam_view
                # Update button text to indicate that the camera can be stopped
                self.left_top_frame_view_nuc_local_cam_button.configure(text="Stop View & Camera ",
                                                                        fg_color=themes["red"])
                rospy.loginfo(f'NUC {nuc_machine} Camera view started successfully!')
            else:
                self.left_top_frame_start_nuc_local_cam_button.configure(text="Stop Camera",
                                                                         fg_color=themes["red"])
                self.left_top_frame_view_only_nuc_local_cam_button.configure(state='normal',
                                                                        fg_color=themes['green'])

        # If camera is running, stop it
        else:
            if show_camera:
                try:
                    self.running_processes[f'nuc{nuc_machine}_view'].shutdown()
                    self.running_processes.pop(f'nuc{nuc_machine}_view', None)
                    rospy.loginfo(f'NUC {nuc_machine} Camera view stopped successfully!')
                except roslaunch.RLException as excep_view:
                    rospy.logerr(
                        f'Error stopping nuc{nuc_machine} camera view: {str(excep_view)}')
                finally:
                    self.left_top_frame_view_nuc_local_cam_button.configure(text="Start & View Camera",
                                                                            fg_color=themes[COLOR_SELECT][0])

            try:
                self.running_processes[f'nuc{nuc_machine}_driver'].shutdown()
                self.running_processes.pop(f'nuc{nuc_machine}_driver', None)
                rospy.loginfo(f'NUC {nuc_machine} Camera stopped successfully!')
            except roslaunch.RLException as excep_camera:
                rospy.logerr(
                    f'Error stopping nuc{nuc_machine} camera driver: {str(excep_camera)}')
            finally:
                # Update button text to indicate that the camera can be started
                self.left_top_frame_start_nuc_local_cam_button.configure(text="Start Camera",
                                                                         fg_color=themes[COLOR_SELECT][0])
                self.left_top_frame_view_only_nuc_local_cam_button.configure(fg_color='gray')
    def _start_camera_calibration(self):
        print('** Starting Camera Calibration **')
        print(f'Board Size: {self.board_size}')
        print(f'Square Size: {self.square_size}')
        self._start_nuc_local_cam_button_event(self.nuc_number, show_camera=False)
        cmd = ['rosrun', 'camera_calibration', 'cameracalibrator.py',
               '--size', self.board_size, '--square', self.square_size, '--k-coefficients=2',
               '--fix-principal-point', 'i', '--fix-aspect-ratio',
               f'image:=/nuc{self.nuc_number}/image_raw', f'camera:=/nuc{self.nuc_number}']

        # Execute the command
        subprocess.call(cmd)
        self._start_nuc_local_cam_button_event(self.nuc_number, show_camera=False)
    def _left_button_frame_calib_update_button_event(self):
        chessboard_entry = self.left_bottom_frame_chessboard_entry.get()
        sq_size_entry = self.left_bottom_frame_sq_size_entry.get()

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
        self.left_button_frame_calib_update_label.configure(text="☑", fg_color='yellow')
        print('Chessboard Parameters Updated!')


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
            self.right_top_frame, text=f"  NUC {self.nuc_number}  ", text_color="yellow",
            bg_color=themes[COLOR_SELECT][1])
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
            self.right_top_frame, text=f'Camera {self.nuc_number}', text_color="white")
        self.right_top_frame_camera_result_label.place(relx=0.72, rely=0.5, anchor="center")
    def _create_right_bottom_frame(self) -> None:
        """_summary_
        """
        self.right_bottom_frame = tk.Frame(self.right_frame, bg=themes[COLOR_SELECT][0])
        self.right_bottom_frame.place(relx=.01, rely=0.14, relwidth=.94, relheight=0.79)

if __name__ == "__main__":
    root = ClientGUI()
    root.mainloop()
