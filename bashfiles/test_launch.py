import roslaunch
import rospkg
import rospy
from os.path import join

class LaunchHandle(object):
    def __init__(self):
        rospy.init_node('launch_handle', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)
        self.package = 'gige_cam_driver'
        self.launch_path ='launch'
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        
        self.cam1_launch_file_path = rospkg.RosPack().get_path('gige_cam_driver') + '/launch/calib1.launch'
        self.cam1_calib = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam1_launch_file_path])
        
        self.cam2_launch_file_path = rospkg.RosPack().get_path('gige_cam_driver') + '/launch/calib2.launch'
        self.cam2_calib = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam2_launch_file_path])
        
        self.cam3_launch_file_path = rospkg.RosPack().get_path('gige_cam_driver') + '/launch/calib3.launch'
        self.cam3_calib = roslaunch.parent.ROSLaunchParent(self.uuid, [self.cam3_launch_file_path])
        
    def on_shutdown(self):
        self.cam1_calib.shutdown()
        self.cam2_calib.shutdown()
        self.cam3_calib.shutdown()
        
    def run(self):
        self.cam1_calib.start()
        self.cam2_calib.start()
        self.cam3_calib.start()
        
if __name__ == '__main__':
    launch_handle = LaunchHandle()
    launch_handle.run()