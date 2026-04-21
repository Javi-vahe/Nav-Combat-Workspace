cmake_minimum_required(VERSION 2.8.3)
project(some_msgs)
set(CMAKE_BUILD_TYPE "Release")
find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    message_generation
    geometry_msgs
)

add_message_files(
  FILES
  Goods.msg
  Motor.msg
  Motors.msg
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
)
