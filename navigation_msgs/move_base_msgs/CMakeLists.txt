cmake_minimum_required(VERSION 2.8.3)
project(move_base_msgs)
# 设置构建类型为Release
set(CMAKE_BUILD_TYPE "Release")
find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    visualization_msgs
    message_generation
    actionlib_msgs
    geometry_msgs
)

add_message_files(
  FILES
  pathpoint.msg
  hgpathplanner.msg
  hglocation.msg
)
add_action_files(
  DIRECTORY
    action
  FILES
    MoveBase.action
)

generate_messages(
  DEPENDENCIES
    std_msgs
    visualization_msgs
    actionlib_msgs
    geometry_msgs
)

catkin_package(
)
