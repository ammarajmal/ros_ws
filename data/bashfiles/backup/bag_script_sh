#!/bin/bash

# Start the roscore
# roscore &
# sleep 2 # Wait for the roscore to start

# Launch the ROS node for aruco_detect
roslaunch aruco_detect cam1.launch
sleep 2 # Wait for the node to start
# Play the bag file
rosbag play /home/ammar/bagfiles/20sec/allcams20sec.bag &

# # Create a CSV file to save the data
csv_file="/home/ammar/data.csv"
echo "time,frame_id,fiducial_id,x,y,z" > $csv_file

# # Start the plot
# gnuplot -e "set xlabel 'time'; set ylabel 'position'; plot '-' using 1:2 with lines title 'x', '-' using 1:3 with lines title 'y', '-' using 1:4 with lines title 'z'"

# # Process the bag file and save data to the CSV file
# rosbag info /home/ammar/bagfiles/20sec/allcams20sec.bag | grep -E "Type: fiducial_msgs/FiducialTransformArray" | awk '{print $2}' | xargs rostopic echo -b /home/ammar/bagfiles/20sec/allcams20sec.bag -p /camera_1_aruco_detect/fiducial_transforms | awk '{print $1","$2","$3","$4.transform.translation.x","$4.transform.translation.y","$4.transform.translation.z}' >> $csv_file

# # Wait for the plot to close
# read -p "Press enter to close the plot"

# # Kill the roscore and ROS node
# killall -9 roscore
# killall -9 roslaunch
