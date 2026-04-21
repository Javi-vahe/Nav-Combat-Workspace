cmake_minimum_required(VERSION 2.8.3)
project(dzsavepath)
# 设置构建类型为Release
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(OpenCV)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  # cv_bridge   
  # image_transport
  geometry_msgs
  tf
)

## Generate messages in the 'msg' folder
#  add_message_files(
#     FILES
#     # Lane.msg
#     # Cluster.msg
#     # ClusterList.msg
#     # Object.msg
#     # ObjectList.msg
  
#  )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES idriverSensorGps
   CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
   message_runtime
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/include/
)

## Declare a C++ executable
 add_executable(dzsavepath src/dzsavepath.cpp src/savepath.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(dzsavepath ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(dzsavepath
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )

