if self.camera_1_active is False:
            # run camera 1
            self.cam1_driver.start()
            self.running_processes.update(
                {"cam1_driver": self.cam1_driver})
            self.camera_1_active = True
            self.sidebar_button_4.configure(text="Stop Camera 1", fg_color=("#fa5f5a", "#ba3732"))
            rospy.sleep(2)
        else:
            self.camera_1_active = False
            self.sidebar_button_4.configure(text="Start Camera 1", fg_color=themes[color_select])
            
            self.running_processes.pop("cam1_driver")
            self.cam1_driver.shutdown()
            print('Camera 1 stopped')
            return