cmake_minimum_required(VERSION 3.0.2)
project(donothing_local_planner)
# 设置构建类型为Release
set(CMAKE_BUILD_TYPE "Release")
## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_core
  tf2_ros
  costmap_2d
  geometry_msgs
  pluginlib
  std_msgs
)

## Declare a catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp nav_core tf2_ros costmap_2d geometry_msgs pluginlib std_msgs
)

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/donothing_local_planner.cpp
)

## Link the library with catkin libraries
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

## Install plugin XML file
install(FILES donothing_local_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
