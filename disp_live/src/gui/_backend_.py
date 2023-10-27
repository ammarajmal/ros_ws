#!/usr/bin/env python3
import rospy
import roslaunch
import subprocess
MACHINE_NAME = 'Main Computer'
themes = {'blue': ("#3B8ED0", "#1F6AA5"),
          'green': ("#2CC985", "#2FA572"),
          'dark-blue': ("#3a7ebf", "#1f538d"),
          'red':("#fa5f5a", "#ba3732")
          }
# select value of COLOR_SELECT from (0: blue, 1: green, 2: dark-blue)
COLOR_SELECT = list(themes.keys())[2]

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

def remote_cam_start_updated(machine_num, remote_nuc_launch,ros_uuid, start_btn, stop_btn):
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
                    start_btn.configure(fg_color=themes['green'])
                    stop_btn.configure(fg_color=themes['red'], state='normal')
            except roslaunch.RLException as e:
                print(f"Error in Starting NUC {machine_num} Camera: {e}")
        else:
            print(f"Camera at NUC {machine_num} is already running..")
    except roslaunch.RLException as e:
        print(e)
    
def remote_cam_stop_updated(machine_num, start_btn, stop_btn):
    """Stops a remote camera at given machine and updates button"""
    try:
        
        if check_active_topic(machine_num):
            print(f"Stopping Camera {machine_num} from {MACHINE_NAME}...")
            kill_ros_node(f"/nuc{machine_num}")
            # processes_[f'nuc{machine_num}_remote_cam_driver'].shutdown()
            # processes_.pop(f'nuc{machine_num}_remote_cam_driver', None)
            rospy.loginfo(f'NUC {machine_num} Camera stopped successfully!')
            start_btn.configure(fg_color=themes[COLOR_SELECT][0])
            stop_btn.configure(fg_color=themes[COLOR_SELECT][0], state = 'disabled')
            
        else:
            print(f"Camera at NUC {machine_num} is already stopped..")
    except roslaunch.RLException as e:
        print(e)
def remote_detect_stop(machine_num):
    try:
        if is_node_running(f"/nuc{machine_num}/fiducial_transforms"):
            print(f"Stopping Detection at NUC {machine_num} from {MACHINE_NAME}...")
            kill_ros_node(f"/nuc{machine_num}/fiducial_transforms")
            rospy.loginfo(f'NUC {machine_num} Detection stopped successfully!')
            # start_btn.configure(fg_color=themes['blue'])
            # stop_btn.configure(fg_color=themes['blue'], state='disabled')
        else:
            print(f"Detection at NUC {machine_num} is already stopped..")
    except roslaunch.RLException as e:
        print(e)
def remote_detect_start(machine_num, remote_detect_launch_file, ros_uuid, start_detect_btn, stop_detect_btn):
    if check_active_topic(machine_num):
        print(f"Starting Detection at NUC {machine_num} from {MACHINE_NAME}...")
        try:
            # if not is_node_running(f"/nuc{machine_num}/aruco_detect"): needs to be updated~ 
            if not is_node_running(f"/aruco_detect"):
                print(f"/nuc{machine_num}/fiducial_transforms is not running!")
                detection_launch_args = [f"{remote_detect_launch_file}",
                                        f'launch_nuc:=nuc{machine_num}']
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(detection_launch_args)[0],
                                    detection_launch_args[1:])]
                nuc_remote_detect_driver = roslaunch.parent.ROSLaunchParent(ros_uuid, roslaunch_file)
                nuc_remote_detect_driver.start()
                rospy.sleep(1)
                stop_detect_btn.configure(fg_color=themes['red'], state='normal')
                start_detect_btn.configure(fg_color=themes['green'])
                try:
                    # if is_node_running(f"/nuc{machine_num}/fiducial_transforms"):
                    if is_node_running(f"/aruco_detect"):
                        print(f"/nuc{machine_num}/fiducial_transforms is running!")
                        rospy.loginfo(f'NUC {machine_num} Detection started successfully!')
                        # start_btn.configure(fg_color=themes['green'])
                        # stop_btn.configure(fg_color=themes['red'], state='normal')
                except roslaunch.RLException as e:
                    print(f"Error in Starting NUC {machine_num} Detection: {e}")
        except roslaunch.RLException as e:
            print(e)
    else:
        print(f"Camera at NUC {machine_num} is not running..")
        
            
def return_nuc_status():
    """ return the status of the nuc """
    return is_node_running("/nuc1"), is_node_running("/nuc2"), is_node_running("/nuc3")
if __name__ == "__main__":
    try:
        print(check_active_topic(2))
        if is_node_running("/nuc1"):
            print("Node is running.")
        else:
            print("Node is not running.")
    except ConnectionRefusedError:
        # Handle the connection error here
        print("ROS Master not running")
        # You can log the error or perform other error-handling actions
    else:
        # Code to execute if no exception is raised
        print("Connected successfully to the server")

