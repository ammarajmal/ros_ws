#!/usr/bin/env python  
#!/usr/bin/env python  
import rospy

import math
import tf2_ros
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf2_aruco_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)


    rate = rospy.Rate(10.0)
    pose_list = []
    while not rospy.is_shutdown():
        trans = None
        try:
            # conversion of coordinates from camera_1 to fiducial_24
            trans = tfBuffer.lookup_transform('fiducial_24', 'camera_3', rospy.Time())
            # save (trans.transform.translation) values to a csv file
            pose_list.append(trans.transform.translation)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue
        finally:
            print(trans)
            

        rate.sleep()
    with open('pose_cam3.csv', 'w') as f:
        f.writelines('x' + ',' + 'y' + ',' + 'z' + ',' + 'disp_x' + ',' + 'disp_y' + ',' + 'disp_z' + '\n')
        for i in range(len(pose_list)):
            f.writelines(str(pose_list[i].x)+','+ str(pose_list[i].y)+','+str(pose_list[i].z)+'\n')
            
