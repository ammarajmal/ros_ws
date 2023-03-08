#!/usr/bin/env python3
import roslaunch
import rospkg
import rospy
import time
import subprocess
import pandas as pd
import matplotlib.pyplot as plt
import os


class SingleLaunchFile:
    def __init__():
        rospy.init_node('launch_handle', anonymous=False)
        self.package = package
        self.launch_file = launch_file
        self.args = args
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)

    def start_launch(self):
        node = roslaunch.core.Node(self.package, self.launch_file, args=self.args)
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.parent = roslaunch.parent.ROSLaunchParent(self.uuid, [node])
        launch.start()

if __name__ == '__main__':
    package = "my_package"
    launch_file = "my_launch_file.launch"
    args = ["arg1:=value1", "arg2:=value2"]
    
    slf = SingleLaunchFile()
    slf.start_launch()
