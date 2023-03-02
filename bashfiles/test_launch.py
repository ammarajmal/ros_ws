import roslaunch
import rospkg
import rospy
from os.path import join

class LaunchHandle(object):
    def __init__(self):
        rospy.init_node('launch_handle', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.package = 'gige_cam_driver'
        self.launch_path = rospkg.RosPack().get_path('gige_cam_driver') + '/launch/'
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        self.cam_bagfile_file_path = self.launch_path + 'bag.launch'
        self.cam_launch_file_path = self.launch_path + 'cam.launch'
        self.cam1_calib_file_path = self.launch_path + 'calib1.launch'
        self.cam1_calib = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam1_calib_file_path, ])
        self.cam2_calib_file_path = self.launch_path + 'calib2.launch'
        self.cam2_calib = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam2_calib_file_path])
        self.cam3_calib_file_path = self.launch_path + 'calib3.launch'
        self.cam3_calib = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam3_calib_file_path])


        # running ros_bag launch file for camera_1
        cli_args = [self.cam_bagfile_file_path, 'cam:=camera_1']
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        self.cam1_bagfile = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        # running ros_bag launch file for camera_2
        cli_args = [self.cam_bagfile_file_path, 'cam:=camera_2']
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        self.cam2_bagfile = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        # running ros_bag launch file for camera_3
        cli_args = [self.cam_bagfile_file_path, 'cam:=camera_3']
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
        self.cam3_bagfile = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)

        # running camera node for all cameras
        cam_dict = {'cam': ['camera_1', 'camera_2', 'camera_3'], 'device_id': ['0', '1', '2'], 'calib_file': ['cam1', 'cam2', 'cam3']}

        for i in range(len(cam_dict['cam'])):
            cli_args = [
                self.cam_launch_file_path,
                f"cam:={cam_dict['cam'][i]}",
                f"device_id:={cam_dict['device_id'][i]}",
                f"calib_file:={cam_dict['calib_file'][i]}",
            ]
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], cli_args[1:])]
            node_name = f"cam{i+1}_driver"
            setattr(self, node_name, roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file))

        # cam_dict = {'cam': ['camera_1', 'camera_2', 'camera_3'], 'device_id': ['0', '1', '2'], 'calib_file': ['cam1', 'cam2', 'cam3']}

        # cli_args_cam1 = [
        #     self.cam_launch_file_path,
        #     f"cam:={cam_dict['cam'][0]}",
        #     f"device_id:={cam_dict['device_id'][0]}",
        #     f"calib_file:={cam_dict['calib_file'][0]}",
        # ]

        # roslaunch_file_cam1 = [(roslaunch.rlutil.resolve_launch_arguments(cli_args_cam1)[0], cli_args_cam1[1:])]
        # self.cam1_driver = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file_cam1)



        self.running_launch_files = {}
        
        
    def on_shutdown(self):
        for file in self.running_launch_files.values():
            file.shutdown()
        
    def run(self):
        
        # ======================================
        # Running Camera Drivers
        # ======================================
        
        # # run camera 1
        # self.cam1_driver.start()
        # self.running_launch_files.update({"cam1_driver": self.cam1_driver})
        
        # # # run camera 2
        # self.cam2_driver.start()
        # self.running_launch_files.update({"cam2_driver": self.cam2_driver})
        
        # # # run camera 3
        # self.cam3_driver.start()
        # self.running_launch_files.update({"cam3_driver": self.cam3_driver})
        # rospy.spin()
        
        
        # ======================================
        # Running Camera Calibration
        # ======================================
        
        
        # self.cam1_calib.start()
        # self.running_launch_files.update({"cam1_calib": self.cam1_calib})
        # self.cam2_calib.start()
        # self.running_launch_files.update({"cam2_calib": self.cam2_calib})
        # self.cam3_calib.start()
        # self.running_launch_files.update({"cam3_calib": self.cam3_calib})
        # rospy.spin()
        
        # ======================================
        # Saving Camera Bag Files 
        # ======================================
        
        # bag file saving for camera 1
        
        # self.cam1_driver.start()
        # self.running_launch_files.update({"cam1_driver": self.cam1_driver})
        # self.cam1_bagfile.start()
        # self.running_launch_files.update({"cam1_bagfile": self.cam1_bagfile})
        # rospy.spin()
        
        
        
        # # bag file saving for camera 2
        
        # self.cam2_driver.start()
        # self.running_launch_files.update({"cam2_driver": self.cam2_driver})
        # self.cam2_bagfile.start()
        # self.running_launch_files.update({"cam2_bagfile": self.cam2_bagfile})
        # rospy.spin()
        
        
        # bag file saving for camera 2
        
        self.cam3_driver.start()
        self.running_launch_files.update({"cam3_driver": self.cam3_driver})
        self.cam3_bagfile.start()
        self.running_launch_files.update({"cam3_bagfile": self.cam3_bagfile})
        rospy.spin()

        # self.cam2_bagfile.start()
        # self.running_launch_files.update({"cam2_bagfile": self.cam2_bagfile})

        # self.cam3_bagfile.start()
        # self.running_launch_files.update({"cam3_bagfile": self.cam3_bagfile})
        # rospy.spin() 
        
        # # self.running_launch_files["cam1_calib"].shutdown()

        # self.cam2_bagfile.start()
        # self.running_launch_files.update({"cam2_bagfile": self.cam2_bagfile})
        

        # self.cam3_bagfile.start()
        # self.running_launch_files.update({"cam3_bagfile": self.cam3_bagfile})
        
        
if __name__ == '__main__':
    launch_handle = LaunchHandle()
    launch_handle.run()