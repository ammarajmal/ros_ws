#!/usr/bin/env python3
# coding=utf-8

import rospy
from fast_cam.srv import SetGain

rospy.init_node('test_ser', anonymous=False)
rospy.wait_for_service('/camera_1/set_gain', timeout=10)
try:
    service_proxy = rospy.ServiceProxy('/camera_1/set_gain', SetGain)
    response = service_proxy(50)
    print(response)
except rospy.ServiceException as e:
    print(f"Service call failed: {e}")
    
