#!/usr/bin/env python3
import roslaunch
import rospkg
import rospy
import tkinter
import tkinter.messagebox
import customtkinter

import time

customtkinter.set_appearance_mode("System")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"

class GUI(customtkinter.CTk):
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

        self.record_bag_launch   = f"{self.launch_path}recordbag.launch"
        self.read_bag_launch  = f"{self.launch_path}readbag.launch"
        self.cam_launch    = f"{self.launch_path}cam.launch"
        self.detect_launch = f"{self.detect_launch_path}detect.launch"
        self.running_processes = {}
        
        # configure window
        self.title("Displacement Measurement using ARUCO Marker")
        self.geometry(f"{1100}x{580}")

        # configure grid layout (4x4)
        self.grid_columnconfigure(1, weight=1)
        self.grid_columnconfigure((2, 3), weight=0)
        self.grid_rowconfigure((0, 1, 2), weight=1)
        
                # create sidebar frame with widgets
        self.sidebar_frame = customtkinter.CTkFrame(self, width=140, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, rowspan=4, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(4, weight=1)
        self.logo_label = customtkinter.CTkLabel(self.sidebar_frame, text="Camera Calibration", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))
        self.sidebar_button_1 = customtkinter.CTkButton(self.sidebar_frame, text='Camera 1', command=lambda: self.sidebar_button_event(1))
        self.sidebar_button_1.grid(row=1, column=0, padx=20, pady=10)
        self.sidebar_button_2 = customtkinter.CTkButton(self.sidebar_frame, text='Camera 2', command=lambda: self.sidebar_button_event(1))
        self.sidebar_button_2.grid(row=2, column=0, padx=20, pady=10)
        self.sidebar_button_3 = customtkinter.CTkButton(self.sidebar_frame, text='Camera 3', command=lambda: self.sidebar_button_event(1))
        self.sidebar_button_3.grid(row=3, column=0, padx=20, pady=10)
        self.appearance_mode_label = customtkinter.CTkLabel(self.sidebar_frame, text="Appearance Mode:", anchor="w")
        self.appearance_mode_label.grid(row=5, column=0, padx=20, pady=(10, 0))
        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(self.sidebar_frame, values=["Light", "Dark", "System"],
                                                                       command=self.change_appearance_mode_event)
        self.appearance_mode_optionemenu.grid(row=6, column=0, padx=20, pady=(10, 10))
        self.scaling_label = customtkinter.CTkLabel(self.sidebar_frame, text="UI Scaling:", anchor="w")
        self.scaling_label.grid(row=7, column=0, padx=20, pady=(10, 0))
        self.scaling_optionemenu = customtkinter.CTkOptionMenu(self.sidebar_frame, values=["80%", "90%", "100%", "110%", "120%"],
                                                               command=self.change_scaling_event)
        self.scaling_optionemenu.grid(row=8, column=0, padx=20, pady=(10, 20))

        # create main entry and button
        self.entry = customtkinter.CTkEntry(self, placeholder_text="Duration")
        self.entry.grid(row=3, column=1, columnspan=2, padx=(20, 0), pady=(20, 20), sticky="nsew")

        self.main_button_1 = customtkinter.CTkButton(master=self,text='EXIT',command= self.exit_button_click , fg_color="transparent", border_width=2, text_color=("gray10", "#DCE4EE"))
        self.main_button_1.grid(row=3, column=3, padx=(20, 20), pady=(20, 20), sticky="nsew")
        
        
        
    def open_input_dialog_event(self):
        dialog = customtkinter.CTkInputDialog(text="Type in a number:", title="CTkInputDialog")
        print("CTkInputDialog:", dialog.get_input())

    def change_appearance_mode_event(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)

    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 100
        customtkinter.set_widget_scaling(new_scaling_float)

    def sidebar_button_event(self, camera_num):
        """Launches the camera calibration process for the camera 1"""
        print("="*36,"      Starting Camera Calibration","="*36, sep="\n" )
        # Launch parent processes for each camera calibration
        
        launch_files = [f"{self.launch_path}calib{num}.launch" for num in range(1, 4)]
        calibrations = {f"cam{num}_calib": roslaunch.parent.ROSLaunchParent(self.uuid, [launch_files[num-1]]) for num in range(1, 4)}
        
        # Start calibration process and wait for it to finish
        calibration = calibrations[f"cam{camera_num}_calib"]
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