#!/usr/bin/env python3
# themes = {'blue': ("#3B8ED0", "#1F6AA5"),
#           'green': ("#2CC985", "#2FA572"),
#           'dark-blue': ("#3a7ebf", "#1f538d")
# }

# color_select = list(themes.keys())[2]  # selection: (0: 'blue') (1: 'green') (2: 'dark-blue')


# print(color_select)


# print(themes[color_select])
running_cams = ['camera_3_driver', 'camera_2_driver']

camera_name_1 = running_cams[0].replace('_driver', '')
camera_name_2 = running_cams[1].replace('_driver', '')
threecameras = f'camera_{camera_name_1.split("_")[1]}{camera_name_2.split("_")[1]}'
print(threecameras)
print(camera_name_1.split('_')[1])
print(camera_name_2.split('_')[1])


                   
                    camera_name_1 = running_cams[0].replace('_driver', '')
                    camera_name_2 = running_cams[1].replace('_driver', '')
                    camera_name_3 = running_cams[2].replace('_driver', '')
                    print("Running Cameras:", camera_name_1, camera_name_2, camera_name_3)
                    threecameras = f'camera_{camera_name_1.split("_")[1]}{camera_name_2.split("_")[1]}{camera_name_3.split("_")[1]}'
                    record_3_cam_launch_args = [
                        self.record_3bag_launch,
                        f'cam1:={camera_name_1}',
                        f'cam2:={camera_name_2}',
                        f'cam3:={camera_name_3}',
                        f'filename:={threecameras}',
                        f'dur:={self.multi_camera_dur}',
                        f'bagfile_datetime:={self.recorded_datetime_var}'
                    ]
                    record_tripple_cam_file = [(roslaunch.rlutil.resolve_launch_arguments(
                        record_3_cam_launch_args)[0], record_3_cam_launch_args[1:])]
                    record_tripple_cam = roslaunch.parent.ROSLaunchParent(
                        self.uuid, record_tripple_cam_file)
                    
                    record_tripple_cam.start()
                    self.running_processes[f'{threecameras}_record'] = record_tripple_cam
                    rec_time = 0
                    while record_tripple_cam.pm.is_alive():
                        rospy.sleep(1)
                        rec_time += 1
                        if rec_time <= int(self.multi_camera_dur):
                            print(
                                f"\033[93mRecording from {threecameras}...{rec_time}/{self.multi_camera_dur}s\033[0m")
                    print(f'Finished recording from {threecameras}')
                    self.running_processes[f'{camera_name_1}_driver'].shutdown()
                    self.running_processes[f'{camera_name_2}_driver'].shutdown()
                    self.running_processes[f'{camera_name_3}_driver'].shutdown()
                    self.running_processes[f'{threecameras}_record'].shutdown()
                    self.running_processes.pop(f'{camera_name_1}_driver')
                    self.running_processes.pop(f'{camera_name_2}_driver')
                    self.running_processes.pop(f'{camera_name_3}_driver')
                    self.running_processes.pop(f'{threecameras}_record')
                    self.multi_camera_active = False
                    self.multi_camera_record_button.configure(
                    text=f"Record", fg_color=themes[COLOR_SELECT])
                    print('Status of self.running_processes: ',self.running_processes)
                    return
