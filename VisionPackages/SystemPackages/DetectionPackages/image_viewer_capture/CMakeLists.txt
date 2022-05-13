cmake_minimum_required(VERSION 3.0.2)
project(image_viewer_capture)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation 
  sensor_msgs
  std_msgs
  pcl_conversions
  pcl_ros
  roscpp
)

add_service_files(
  FILES
  GetFrame.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS 
  cv_bridge 
  image_transport 
  message_runtime 
  sensor_msgs
  std_msgs
  pcl_conversions 
  pcl_ros
)

find_package(OpenCV)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

# add the subscriber example
add_executable(multiastra_subscriber src/multiastra_subscriber.cpp)
add_dependencies(multiastra_subscriber ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(multiastra_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})