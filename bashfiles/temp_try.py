#!/usr/bin python3
import roslaunch
import rospy
import rospkg
import customtkinter
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
    """GUI class for the server side of the application
    """
    def __init__(self) -> None:
        super().__init__()
        rospy.init_node("gui_launch_handle", anonymous=False)
        self.camera_driver_package = 'gige_cam_driver'
        self.aruco_detect_package = 'aruco_detect'

        # path management
        self.launch_cam_path = f'{rospkg.RosPack().get_path(self.camera_driver_package)}/launch/'
        self.csv_folder_path = f'{rospkg.RosPack().get_path(self.camera_driver_package)}/csvfiles/'
        self.detect_launch_path = f'{rospkg.RosPack().get_path(self.aruco_detect_package)}/launch/'
        
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        
        # self.camera1_launch = f''
        