#!/usr/bin/env python3
import subprocess
import socket
import re
import rospy
import roslaunch


MACHINE_NAME = 'Main Computer'
themes = {'blue': ("#3B8ED0", "#1F6AA5"),
          'green': ("#2CC985", "#2FA572"),
          'dark-blue': ("#3a7ebf", "#1f538d"),
          'red':("#fa5f5a", "#ba3732")
          }
# select value of COLOR_SELECT from (0: blue, 1: green, 2: dark-blue)
COLOR_SELECT = list(themes.keys())[2]
def get_ros_topic_frequency(topic):
    """Start the rostopic hz command"""
    process = subprocess.Popen(['rostopic', 'hz', topic], stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    # Wait a bit to get some output
    rospy.sleep(0.9)

    # Terminate the process
    process.terminate()

    # Read the output
    try:
        output, _ = process.communicate(timeout=2)
        output = output.decode('utf-8')
    except subprocess.TimeoutExpired:
        process.kill()
        output, _ = process.communicate()

    # Extract frequency from the output using regular expression
    match = re.search(r'average rate: ([\d\.]+)', output)
    if match:
        return float(match.group(1))
    else:
        return None
def _check_ros_status_function(status_btn):
    active_systems = str()
    my_topic = f"/camera_{nuc_machine}/image_raw"
    print(_topic_check(my_topic))
    topic_name = f"/camera_{nuc_machine}/image_raw"
    
    camera_1 = _topic_check(f"/camera_1/image_raw")
    camera_2 = _topic_check(f"/camera_2/image_raw")
    camera_3 = _topic_check(f"/camera_3/image_raw")
    
    
    # camera_1 = check_active_topic(1)
    # camera_2 = check_active_topic(2)
    # camera_3 = check_active_topic(3)
    # Now, check which of the camera_1, camera_2, and nuc3 is True, add to active_systems
    if camera_1 is True:
        active_systems += '     NUC 1'
    if camera_2 is True:
        active_systems += '     NUC 2'
    if camera_3 is True:
        active_systems += '     NUC 3'
    if camera_1 is False and camera_2 is False and camera_3 is False:
        active_systems = "No Active Camera"
    print(active_systems)
    status_btn.configure(text=active_systems)

def is_node_running(node_name):
    """ Checks if a ROS node is running"""
    try:
        # Use the subprocess module to run the 'rosnode list' command
        # and capture its outputis_node_running
        output = subprocess.check_output(['rosnode', 'list'], universal_newlines=True)

        # Check if the node name is in the list of running nodes
        return node_name in output.split('\n')
    except subprocess.CalledProcessError:
        # Handle any errors that occur when running the command
        return False

def kill_ros_node(node_name):
    """ Kills a ROS node"""
    try:
        # Use the subprocess module to run the 'rosnode kill' command
        subprocess.call(['rosnode', 'kill', node_name])
    except subprocess.CalledProcessError:
        # Handle any errors that occur when running the command
        print(f"Failed to kill node {node_name}")
# def detection_start(nuc_number, detect_launch, uuid, self.marker_dim, self.marker_dict)
def detection_start(machine_number, launch_file, ros_uuid, marker_dimension, maker_dictionary):
    """ Starts the detection node on the given machine """
    print(f'Entered detection_start function with {machine_number}')
    try:
        if is_node_running(f"/camera_{machine_number}"):
            print(f" Starting Detection at NUC {machine_number}!")
            detection_launch_args = [f"{launch_file}",
                                     f'launch_nuc:=camera_{machine_number}',
                                     f'fiducial_len:={marker_dimension}',
                                     f'dictionary:={maker_dictionary}']
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(detection_launch_args)[0],
                               detection_launch_args[1:])]
            nuc_detect_driver = roslaunch.parent.ROSLaunchParent(ros_uuid, roslaunch_file)
            nuc_detect_driver.start()
            rospy.sleep(1)
            try:
                if is_node_running(f"/camera_{machine_number}/fiducial_transforms"):
                    print(f"/camera_{machine_number}/fiducial_transforms is running!")
                    rospy.loginfo(f'NUC {machine_number} Detection started successfully!')
            except roslaunch.RLException as e:
                print(f"Error in Starting NUC {machine_number} Detection: {e}")
        else:
            rospy.logerr(f"Camera at NUC {machine_number} is not running..")
    except roslaunch.RLException as e:
        print(e)

def detection_stop(machine_number)-> None:
    """ Stops the detection node on the given machine """
    try:
        node_to_kill = f'/camera_{machine_number}/aruco_detect'
        kill_ros_node(node_to_kill)
        rospy.loginfo(f'NUC {machine_number} Detection stopped successfully!')
    except roslaunch.RLException as e:
        print(f'Error stopping the Detection Node, {e}')
        


def check_active_topic(nuc_machine):
    """Checks whether a topic is currently running/active or not.. """
    topic_name = f"/camera_{nuc_machine}/image_raw"
    all_topics = rospy.get_published_topics()
    return topic_name in [topic[0] for topic in all_topics]


def _topic_check(this_topic):
    """Checks whether a given topic is currently running/active or not.. """
    all_topics = rospy.get_published_topics()
    return this_topic in [topic[0] for topic in all_topics]


def remote_cam_start_updated(machine_num, remote_nuc_launch,ros_uuid, start_btn, stop_btn):
    """Starts a remote camera at given machine and updates button"""
    try:
        if not _topic_check(f"/camera_{machine_num}/image_raw"):
        # if not check_active_topic(machine_num):
            print(f"Starting Camera {machine_num} from {MACHINE_NAME}...")
            camera_launch_args = [f"{remote_nuc_launch}",
                                    f'launch_nuc:=camera_{machine_num}']
            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(camera_launch_args)[0],
                                camera_launch_args[1:])]
            nuc_remote_cam_driver = roslaunch.parent.ROSLaunchParent(ros_uuid, roslaunch_file)
            nuc_remote_cam_driver.start()
            rospy.sleep(1) # waiting to camera node to start
            try:
                if is_node_running(f"/camera_{machine_num}"):
                    print(f"/camera_{machine_num} is running!")
                    
                    # processes_[f'camera_{machine_num}_remote_cam_driver'] = nuc_remote_cam_driver
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
        if _topic_check(f"/camera_{machine_num}/image_raw"):
        # if check_active_topic(machine_num):
            print(f"Stopping Camera {machine_num} from {MACHINE_NAME}...")
            kill_ros_node(f"/camera_{machine_num}")
            # processes_[f'camera_{machine_num}_remote_cam_driver'].shutdown()
            # processes_.pop(f'camera_{machine_num}_remote_cam_driver', None)
            rospy.loginfo(f'NUC {machine_num} Camera stopped successfully!')
            start_btn.configure(fg_color=themes[COLOR_SELECT][0])
            stop_btn.configure(fg_color=themes[COLOR_SELECT][0], state = 'disabled')
            
        else:
            print(f"Camera at NUC {machine_num} is already stopped..")
    except roslaunch.RLException as e:
        print(e)
def remote_detect_stop(machine_num):
    try:
        if is_node_running(f"/camera_{machine_num}/fiducial_transforms"):
            print(f"Stopping Detection at NUC {machine_num} from {MACHINE_NAME}...")
            kill_ros_node(f"/camera_{machine_num}/fiducial_transforms")
            rospy.loginfo(f'NUC {machine_num} Detection stopped successfully!')
            # start_btn.configure(fg_color=themes['blue'])
            # stop_btn.configure(fg_color=themes['blue'], state='disabled')
        else:
            print(f"Detection at NUC {machine_num} is already stopped..")
    except roslaunch.RLException as e:
        print(e)
def remote_detect_start(machine_num, remote_detect_launch_file, ros_uuid, start_detect_btn, stop_detect_btn):
    if _topic_check(f"/camera_{machine_num}/image_raw"):
    # if check_active_topic(machine_num):
        print(f"Starting Detection at NUC {machine_num} from {MACHINE_NAME}...")
        try:
            # if not is_node_running(f"/camera_{machine_num}/aruco_detect"): needs to be updated~ 
            if not is_node_running("/aruco_detect"):
                print(f"/camera_{machine_num}/fiducial_transforms is not running!")
                detection_launch_args = [f"{remote_detect_launch_file}",
                                        f'launch_nuc:=camera_{machine_num}']
                roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(detection_launch_args)[0],
                                    detection_launch_args[1:])]
                nuc_remote_detect_driver = roslaunch.parent.ROSLaunchParent(ros_uuid, roslaunch_file)
                nuc_remote_detect_driver.start()
                rospy.sleep(1)
                stop_detect_btn.configure(fg_color=themes['red'], state='normal')
                start_detect_btn.configure(fg_color=themes['green'])
                try:
                    # if is_node_running(f"/camera_{machine_num}/fiducial_transforms"):
                    if is_node_running("/aruco_detect"):
                        print(f"/camera_{machine_num}/fiducial_transforms is running!")
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
    return is_node_running("/camera_1"), is_node_running("/camera_2"), is_node_running("/camera_3")
def get_lan_ip():
    """ Get the local IP address of the machine """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # doesn't even have to be reachable
        s.connect(('8.8.8.8', 1))
        ip_addr = s.getsockname()[0]
    except socket.error:
        ip_addr = '127.0.0.1'
    finally:
        s.close()
    return ip_addr



if __name__ == "__main__":
    nuc_machine = 2
    my_topic = f"/camera_{nuc_machine}/image_raw"
    print(_topic_check(my_topic))
    try:
        print(_topic_check(f"/camera_2/image_raw"))
        print(check_active_topic(2))
        if is_node_running("/camera_1"):
            print("Node is running.")
        else:
            print("Node is not running.")
    except ConnectionRefusedError:
        # Handle the connection error here
        print("ROS Master @camera_1 not running")
        # You can log the error or perform other error-handling actions
    else:
        # Code to execute if no exception is raised
        print("Connected successfully to the server")

    lan_ip = get_lan_ip()
    print("LAN IP Address:", lan_ip)
