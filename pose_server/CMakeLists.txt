cmake_minimum_required(VERSION 2.8.3)
project(pose_server)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs  
  tf2
  tf2_ros
)
find_package(OpenCV)

add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
  ObjectCount.msg
  Pose.msg
  PoseList.msg
)

add_service_files(
  FILES
  ImgToDepth.srv
  PoseSrv.srv
  PoseZividSrv.srv
  Capture.srv
  CaptureAssistantSuggestSettings.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pose_server
  CATKIN_DEPENDS message_runtime pcl_conversions pcl_ros roscpp sensor_msgs std_msgs geometry_msgs tf2 tf2_ros
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Zed
add_library(depth_srv_zed src/zed/depth_srv_zed.cpp)
add_executable(depth_main_zed src/zed/depth_main_zed.cpp)
add_dependencies(depth_main_zed ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(depth_main_zed 
    depth_srv_zed
    ${catkin_LIBRARIES}
)

add_library(pose_srv_zed src/zed/pose_srv_zed.cpp)
add_executable(pose_main_zed src/zed/pose_main_zed.cpp)
add_dependencies(pose_main_zed ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pose_main_zed
    pose_srv_zed    
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

## Zivid 
add_library(depth_srv_zivid src/zivid/depth_srv_zivid.cpp)
add_executable(depth_main_zivid src/zivid/depth_main_zivid.cpp)
add_dependencies(depth_main_zivid ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(depth_main_zivid 
    depth_srv_zivid
    ${catkin_LIBRARIES}
)

add_library(pose_srv_zivid src/zivid/pose_srv_zivid.cpp)
add_executable(pose_main_zivid src/zivid/pose_main_zivid.cpp)
add_dependencies(pose_main_zivid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pose_main_zivid
    pose_srv_zivid    
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

## Astra
add_library(depth_srv_astra src/astra/depth_srv_astra.cpp)
add_executable(depth_main_astra src/astra/depth_main_astra.cpp)
add_dependencies(depth_main_astra ${${PROJECT_NAME}_EXPORTED_TARGETS})

target_link_libraries(depth_main_astra 
    depth_srv_astra
    ${catkin_LIBRARIES}
)

add_library(pose_srv_astra 
    src/astra/pose_srv_astra.cpp
    src/astra/pose_srv_astra_cv.cpp
    src/astra/single_item.cpp
    src/astra/multi_items.cpp
    src/astra/utils.cpp
)
add_executable(pose_main_astra src/astra/pose_main_astra.cpp)
add_dependencies(pose_main_astra ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(pose_main_astra
    pose_srv_astra    
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
