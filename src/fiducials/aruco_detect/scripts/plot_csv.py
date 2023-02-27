#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
x_values = []
y_values = []
z_values = []
x_val = float()
with open('/home/agcam/ros_ws/src/fiducials/aruco_detect/scripts/fiducial_transforms.csv') as csv_file:
    csv_reader = csv.DictReader(csv_file)
    for row in csv_reader:
        x_val = (row['x'])
        print(x_val)
        # print(type((row['x'])))
#         x_values.append(float(row['x']))
#         y_values.append(float(row['y']))
#         z_values.append(float(row['z']))
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.plot(x_values, y_values, z_values, '-b')
# plt.show()
