execute_process(COMMAND "/home/ammar/ros_ws/build/gige_cam_driver/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ammar/ros_ws/build/gige_cam_driver/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
