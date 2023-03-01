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
        
        self.cam1_calib_file_path = self.launch_path + 'calib1.launch'
        self.cam1_calib = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam1_calib_file_path])
        
        self.cam1_bagfile_file_path = self.launch_path + 'bag1.launch'
        self.cam1_bagfile = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam1_bagfile_file_path])
        
        
        self.cam2_calib_file_path = self.launch_path + 'calib2.launch'
        self.cam2_calib = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam2_calib_file_path])
        
        self.cam2_bagfile_file_path = self.launch_path + 'bag2.launch'
        self.cam2_bagfile = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam2_bagfile_file_path])
        
        
        self.cam3_calib_file_path = self.launch_path + 'calib3.launch'
        self.cam3_calib = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam3_calib_file_path])
        
        self.cam3_bagfile_file_path = self.launch_path + 'bag3.launch'
        self.cam3_bagfile = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam3_bagfile_file_path])
        
        self.running_launch_files = {}
        
        
    def on_shutdown(self):
        for file in self.running_launch_files.values():
            file.shutdown()
        
    def run(self):
        self.cam1_calib.start()
        self.running_launch_files.update({"cam1_calib": self.cam1_calib})
        self.cam1_bagfile.start()
        self.running_launch_files.update({"cam1_bagfile": self.cam1_bagfile})
        
        # self.running_launch_files["cam1_calib"].shutdown()
        self.cam2_calib.start()
        self.running_launch_files.update({"cam2_calib": self.cam2_calib})
        self.cam2_bagfile.start()
        self.running_launch_files.update({"cam2_bagfile": self.cam2_bagfile})
        
        self.cam3_calib.start()
        self.running_launch_files.update({"cam3_calib": self.cam3_calib})
        self.cam3_bagfile.start()
        self.running_launch_files.update({"cam3_bagfile": self.cam3_bagfile})
        
        
if __name__ == '__main__':
    launch_handle = LaunchHandle()
    launch_handle.run()