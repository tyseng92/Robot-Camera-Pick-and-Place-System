cmake_minimum_required(VERSION 3.0.2)
project(cam_calib)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

## Generate messages in the 'msg' folder
#add_message_files(
#FILES
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opencv_contour
  CATKIN_DEPENDS rospy sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/get_points.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
