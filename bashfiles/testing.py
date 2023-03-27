#!/usr/bin/env python3

# install camera sdk from: https://minsvision.com/rjxz

import datetime
import os
import subprocess
import time

import tkinter as tk
from tkinter import filedialog

import roslaunch
import rospkg
import rospy




class RosbagPlayer:
    def __init__(self, bag_file_path, playback_rate=10):
        self.bag_file_path = bag_file_path
        self.playback_rate = playback_rate
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.launch_file = "rosbag_player.launch"
        self.launch = self.setup_launch()
        
    def setup_launch(self):
        cli_args = [self.bag_file_path, "playback_rate:={}".format(self.playback_rate)]
        node = roslaunch.core.Node(package="rosbag", 
                                    executable="play", 
                                    name="rosbag_player", 
                                    output="log", 
                                    remap_args=["__name:=rosbag_player"])
        launch = roslaunch.scriptapi.ROSLaunch()
        launch.start()
        process = launch.launch(node, args=cli_args)
        return launch
    
    def stop(self):
        self.launch.stop()

class GUI:
    def __init__(self):
        self.window = tk.Tk()
        self.window.title("ROS Bag Player")
        self.window.geometry("500x150")
        
        self.bag_file_path = tk.StringVar()
        self.bag_file_path.set("/home/ammar/ros_ws/src/gige_cam_driver/bagfiles/")
        self.playback_rate = tk.StringVar(value="10")
        
        tk.Label(self.window, text="Select ROS bag file: ").grid(row=0, column=0)
        tk.Entry(self.window, textvariable=self.bag_file_path, width=30).grid(row=0, column=1)
        tk.Button(self.window, text="Browse", command=self.select_bag_file).grid(row=0, column=2)
        tk.Label(self.window, text="Enter playback rate: ").grid(row=1, column=0)
        tk.Entry(self.window, textvariable=self.playback_rate, width=10).grid(row=1, column=1)
        
        tk.Button(self.window, text="Play", command=self.play_bag_file).grid(row=2, column=0, padx=10, pady=10)
        tk.Button(self.window, text="Stop", command=self.stop_bag_file, state="disabled").grid(row=2, column=1, padx=10, pady=10)
        
        self.player = None
        
        self.window.mainloop()
        
    def select_bag_file(self):
        file_path = filedialog.askopenfilename(initialdir=self.bag_file_path.get(), title="Select ROS bag file")
        self.bag_file_path.set(file_path)
        
    def play_bag_file(self):
        bag_file_path = self.bag_file_path.get()
        playback_rate = int(self.playback_rate.get())
        self.player = RosbagPlayer(bag_file_path, playback_rate)
        tk.Button(self.window, text="Play", command=self.play_bag_file, state="disabled").grid(row=2, column=0, padx=10, pady=10)
        tk.Button(self.window, text="Stop", command=self.stop_bag_file).grid(row=2, column=1, padx=10, pady=10)
        
    def stop_bag_file(self):
        if self.player:
            self.player.stop()
            self.player = None

if __name__ == '__main__':
    gui = GUI()
