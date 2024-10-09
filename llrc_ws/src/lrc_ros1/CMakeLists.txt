cmake_minimum_required(VERSION 3.0.2)
project(lrc_ros1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_conversions
  pcl_ros
  radar_msgs
)
find_package(PCL REQUIRED)
find_package(OpenCV)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp rospy std_msgs radar_msgs
)

include_directories(
  include/lrc_ros1
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)
include_directories(${CMAKE_SOURCE_DIR}/lrc_ros1)

add_executable(lrc src/lrc.cpp)
target_link_libraries(lrc
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${cv_bridge_LIBRARIES} 
  ${sensor_msgs_LIBRARIES}
)