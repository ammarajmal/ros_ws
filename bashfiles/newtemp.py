#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
def get_camera_name(string_cam):
    try:
        camera_name =  "_".join(string_cam.split('/')[-1].split('_')[:2])
        complete_filename = string_cam.split('/')[-1].split('.')[0]
        return [camera_name, complete_filename]
    except:
        return None
def csv_plotting(csv_file):
        """This function plots the data from the csv file passed as input argument"""
        camera, path = get_camera_name(csv_file)
        data = pd.read_csv(csv_file)
        jpg_file = csv_file.replace('.csv', '.png')
        # Extract the columns from the csv file
        camera_name = data['field.header.frame_id']
        marker_id = data['field.transforms0.fiducial_id']
        image_seq = data['field.image_seq']
        x_disp = data['field.transforms0.transform.translation.x'] - data['field.transforms0.transform.translation.x'][0]
        y_disp = data['field.transforms0.transform.translation.y'] - data['field.transforms0.transform.translation.y'][0]
        z_disp = data['field.transforms0.transform.translation.z'] - data['field.transforms0.transform.translation.z'][0]
        # finding the length of x_disp
        # len_disp = len(x_disp)
        time_vector = range(len(x_disp))
        print(time_vector)

        fig, axis_plot = plt.subplots(nrows=3, ncols=2)

        # set the title of the figure
        fig.suptitle(f'3D displacement using ARUCO marker detection - {camera}')

        # Plotting the x, y, z displacement
        axis_plot[0, 0].plot(time_vector, x_disp, 'r')
        axis_plot[0, 0].set_title('X displacement')

        axis_plot[1, 0].plot(time_vector, y_disp, 'g')
        axis_plot[1, 0].set_title('Y displacement')

        axis_plot[2, 0].plot(time_vector, z_disp, 'b')
        axis_plot[2, 0].set_title('Z displacement')
        
        axis_plot[0, 0].grid(True)
        axis_plot[1, 0].grid(True)
        axis_plot[2, 0].grid(True)

        # Saving the plot with a given name
        fig.savefig(jpg_file)
        print('Close the figure to continue..')
        plt.show()
if __name__ == '__main__':
    file_ = '/home/agcam/ros_ws/src/gige_cam_driver/csvfiles/backup/camera_1_50s_2023-04-05_08-44-37.csv'
    csv_plotting(file_)