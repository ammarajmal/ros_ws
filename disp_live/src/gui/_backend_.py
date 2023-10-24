#!/usr/bin/env python3
import rospy
import roslaunch
MACHINE_NAME = 'Main Computer'



def check_active_topic(nuc_machine):
    """Checks whether a topic is currently running/active or not.. """
    topic_name = f"/nuc{nuc_machine}/image_raw"
    all_topics = rospy.get_published_topics()
    return topic_name in [topic[0] for topic in all_topics]

def remote_cam_start_(machine_num, remote_nuc_launch,ros_uuid, processes_, button_):
    """Starts a remote camera at given machine and updates button"""
    try:
        if not check_active_topic(machine_num):
            print(f"Starting Camera {machine_num} from {MACHINE_NAME}...")
            camera_launch_args = [f"{remote_nuc_launch}",
                                    f'launch_nuc:=nuc{machine_num}']
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(camera_launch_args)[0],
                                camera_launch_args[1:])]
            nuc_remote_cam_driver = roslaunch.parent.ROSLaunchParent(ros_uuid, roslaunch_file)
            nuc_remote_cam_driver.start()
            processes_[f'nuc{machine_num}_remote_cam_driver'] = nuc_remote_cam_driver
            rospy.loginfo(f'NUC {machine_num} Camera started successfully!')
            button_(machine_num, 'started')
        else:
            print('Camera already running..')
            try:
                processes_[f'nuc{machine_num}_remote_cam_driver'].shutdown()
                processes_.pop(f'nuc{machine_num}_remote_cam_driver', None)
                rospy.loginfo(f'NUC {machine_num} Camera stopped successfully!')
            except:
                rospy.logerr(
                    f'Error stopping nuc{machine_num} camera driver: {str(excep_camera)}')
            finally:
                # Update button text to indicate that the camera can be started
                button_(machine_num, 'stopped')
    except roslaunch.RLException as e:
        print(e)
    
if __name__ == "__main__":
    print(check_active_topic(2))