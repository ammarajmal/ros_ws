[Desktop Entry]
Name=GUI
Comment=GUI for NUC


Exec=gnome-terminal -- /bin/bash -c "source /opt/ros/noetic/setup.bash && source /home/sitl3/ros_ws/devel/setup.bash && cd /home/sitl3/ros_ws/disp_live/src/gui && python3 Updated_nuc_gui_3.py; exec /bin/bash"



Icon=/home/sitl3/ros_ws/disp_live/src/gui/GUI.png
Terminal=true
Type=Application
Categories=Utility;Application;


