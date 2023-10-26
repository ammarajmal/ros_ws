#!/usr/bin/env python3
import rospy
import roslaunch
import subprocess
MACHINE_NAME = 'Main Computer'

def is_node_running(node_name):
    try:
        # Use the subprocess module to run the 'rosnode list' command
        # and capture its output
        output = subprocess.check_output(['rosnode', 'list'], universal_newlines=True)

        # Check if the node name is in the list of running nodes
        return node_name in output.split('\n')
    except subprocess.CalledProcessError:
        # Handle any errors that occur when running the command
        return False

def kill_ros_node(node_name):
    try:
        # Use the subprocess module to run the 'rosnode kill' command
        subprocess.call(['rosnode', 'kill', node_name])
    except subprocess.CalledProcessError:
        # Handle any errors that occur when running the command
        print(f"Failed to kill node {node_name}")




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
            rospy.sleep(1) # waiting to camera node to start
            try:
                if is_node_running(f"/nuc{machine_num}"):
                    print(f"/nuc{machine_num} is running!")
                    
                    processes_[f'nuc{machine_num}_remote_cam_driver'] = nuc_remote_cam_driver
                    rospy.loginfo(f'NUC {machine_num} Camera started successfully!')
                    button_(machine_num, 'started')
            except Exception as excep_camera:
                print(f"/nuc{machine_num} is NOT running!")
                rospy.logerr(
                    f'Error starting nuc{machine_num} camera driver: {str(excep_camera)}')
        else:
            try:
                processes_[f'nuc{machine_num}_remote_cam_driver'].shutdown()
                processes_.pop(f'nuc{machine_num}_remote_cam_driver', None)
                rospy.loginfo(f'NUC {machine_num} Camera stopped successfully!')
            except Exception as excep_camera:
                print('Camera already running..')
            finally:
               # Update button text to indicate that the camera can be started
                button_(machine_num, 'stopped')
    except roslaunch.RLException as e:
        print(e)
def remote_cam_start_updated(machine_num, remote_nuc_launch,ros_uuid, processes_, button_, start_btn, stop_btn):
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
            rospy.sleep(1) # waiting to camera node to start
            try:
                if is_node_running(f"/nuc{machine_num}"):
                    print(f"/nuc{machine_num} is running!")
                    
                    # processes_[f'nuc{machine_num}_remote_cam_driver'] = nuc_remote_cam_driver
                    rospy.loginfo(f'NUC {machine_num} Camera started successfully!')
                    start_btn.configure(fg_color='green')
                    stop_btn.configure(fg_color='red')
            except roslaunch.RLException as e:
                print(f"Error in Starting NUC {machine_num} Camera: {e}")
        else:
            print(f"Camera at NUC {machine_num} is already running..")
            # try:
            #     processes_[f'nuc{machine_num}_remote_cam_driver'].shutdown()
            #     processes_.pop(f'nuc{machine_num}_remote_cam_driver', None)
            #     rospy.loginfo(f'NUC {machine_num} Camera stopped successfully!')
            # except Exception as excep_camera:
            #     print('Camera already running..')
            # finally:
            #    # Update button text to indicate that the camera can be started
            #     button_(machine_num, 'stopped')
    except roslaunch.RLException as e:
        print(e)







def return_nuc_status():
    """ return the status of the nuc """
    return is_node_running("/nuc1"), is_node_running("/nuc2"), is_node_running("/nuc3")
if __name__ == "__main__":
    print(check_active_topic(2))
    if is_node_running("/nuc1"):
        print("Node is running.")
    else:
        print("Node is not running.")
