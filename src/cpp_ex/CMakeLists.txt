cmake_minimum_required(VERSION 3.0.2)
project(cpp_ex)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cpp_ex
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
add_executable(subscriber_cpp src/subscriber_cpp.cpp)
target_link_libraries(subscriber_cpp ${catkin_LIBRARIES})

add_executable(publisher_cpp src/publisher_cpp.cpp)
target_link_libraries(publisher_cpp ${catkin_LIBRARIES})


