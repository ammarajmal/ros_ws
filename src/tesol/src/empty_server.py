#!/usr/bin/env python3
""" ros node for service server"""
from std_srvs.srv import Empty, EmptyResponse
import rospy

def empty_response_callback(req):
    """ service callback"""
    print("!!!   Empty Service !!!")
    return EmptyResponse()

if __name__ == "__main__":
    rospy.logwarn("!!!   Starting Empty Service !!!")
    rospy.init_node("empty_server_service")
    srv = rospy.Service("print_Service", Empty, empty_response_callback)
    rospy.spin()
