# CMake generated Testfile for 
# Source directory: /home/ammar/ros_ws/src/image_pipeline_custom/camera_calibration
# Build directory: /home/ammar/ros_ws/build/camera_calibration
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_camera_calibration_nosetests_test.directed.py "/home/ammar/ros_ws/build/camera_calibration/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/ammar/ros_ws/build/camera_calibration/test_results/camera_calibration/nosetests-test.directed.py.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/ammar/ros_ws/build/camera_calibration/test_results/camera_calibration" "/usr/bin/nosetests3 -P --process-timeout=60 /home/ammar/ros_ws/src/image_pipeline_custom/camera_calibration/test/directed.py --with-xunit --xunit-file=/home/ammar/ros_ws/build/camera_calibration/test_results/camera_calibration/nosetests-test.directed.py.xml")
set_tests_properties(_ctest_camera_calibration_nosetests_test.directed.py PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/ammar/ros_ws/src/image_pipeline_custom/camera_calibration/CMakeLists.txt;11;catkin_add_nosetests;/home/ammar/ros_ws/src/image_pipeline_custom/camera_calibration/CMakeLists.txt;0;")
add_test(_ctest_camera_calibration_nosetests_test.multiple_boards.py "/home/ammar/ros_ws/build/camera_calibration/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/ammar/ros_ws/build/camera_calibration/test_results/camera_calibration/nosetests-test.multiple_boards.py.xml" "--return-code" "\"/usr/bin/cmake\" -E make_directory /home/ammar/ros_ws/build/camera_calibration/test_results/camera_calibration" "/usr/bin/nosetests3 -P --process-timeout=60 /home/ammar/ros_ws/src/image_pipeline_custom/camera_calibration/test/multiple_boards.py --with-xunit --xunit-file=/home/ammar/ros_ws/build/camera_calibration/test_results/camera_calibration/nosetests-test.multiple_boards.py.xml")
set_tests_properties(_ctest_camera_calibration_nosetests_test.multiple_boards.py PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/catkin/cmake/test/nosetests.cmake;83;catkin_run_tests_target;/home/ammar/ros_ws/src/image_pipeline_custom/camera_calibration/CMakeLists.txt;24;catkin_add_nosetests;/home/ammar/ros_ws/src/image_pipeline_custom/camera_calibration/CMakeLists.txt;0;")
subdirs("gtest")
