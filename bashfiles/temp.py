#!/usr/bin/env python3
# themes = {'blue': ("#3B8ED0", "#1F6AA5"),
#           'green': ("#2CC985", "#2FA572"),
#           'dark-blue': ("#3a7ebf", "#1f538d")
# }

# color_select = list(themes.keys())[2]  # selection: (0: 'blue') (1: 'green') (2: 'dark-blue')


# print(color_select)


# print(themes[color_select])


def camera_button_event_3(self):
    """This function is called when a camera button is pressed"""
    self.cli_args = [
        self.cam_launch,
        'cam:=camera_3',
        'device_id:=2',
        'calib_file:=cam3',
    ]
    self.roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(self.cli_args)[0], self.cli_args[1:])]
    setattr(self, 'cam3_driver', roslaunch.parent.ROSLaunchParent(self.uuid, self.roslaunch_file))

    if self.camera_3_active is False:
        # run camera 3
        self.camera_3_active = True
        self.sidebar_button_6.configure(text="Stop Camera 3", fg_color=("#fa5f5a", "#ba3732"))
        self.cam3_driver.start()
        self.running_processes.update(
            {"cam3_driver": self.cam3_driver})
        rospy.sleep(2)
    else:
        self.cam3_driver.shutdown()
        self.running_processes.pop("cam3_driver")
        print('Camera 3 stopped')
        self.camera_3_active = False
        self.sidebar_button_6.configure(text="Start Camera 3", fg_color=themes[color_select])
        return