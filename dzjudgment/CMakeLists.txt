cmake_minimum_required(VERSION 2.8.3)
project(dzjudgment)
# 设置构建类型为Release
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O1")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg serial tf)


## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(   
   CATKIN_DEPENDS roscpp rospy std_msgs
   message_runtime
   serial)

## Build dzjudgment
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(
    dzjudgment
    src/dzjudgment.cpp
    src/judgment.cpp
  )
target_link_libraries(dzjudgment ${catkin_LIBRARIES})
add_dependencies(dzjudgment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

