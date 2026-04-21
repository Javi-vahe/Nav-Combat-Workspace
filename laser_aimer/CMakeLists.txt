cmake_minimum_required(VERSION 3.0.2)
project(laser_aimer)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  tf
  tf2_ros
  tf2_sensor_msgs
  geometry_msgs
  pcl_ros
  pcl_conversions
  laser_geometry
  nav_msgs
  message_filters
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES laser_aimer
  # CATKIN_DEPENDS roscpp sensor_msgs tf tf2 tf2_ros geometry_msgs pcl_ros pcl_conversions laser_geometry
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(laser_aimer_node src/laser_aimer_node.cpp)
target_link_libraries(laser_aimer_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(point_sync_node src/point_sync_node.cpp)
target_link_libraries(point_sync_node ${catkin_LIBRARIES})