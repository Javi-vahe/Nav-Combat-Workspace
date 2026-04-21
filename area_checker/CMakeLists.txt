cmake_minimum_required(VERSION 3.0.2)
project(area_checker)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  visualization_msgs
  tf
  std_msgs
)
find_package(yaml-cpp REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES area_checker
#  CATKIN_DEPENDS geometry_msgs roscpp visualization_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(polygon_checker_node src/polygon_checker.cpp)
target_link_libraries(polygon_checker_node
  ${catkin_LIBRARIES}
)

add_executable(area_checker_node src/area_checker.cpp)
target_link_libraries(area_checker_node
  ${catkin_LIBRARIES}
)

add_executable(area_recorder_node src/area_recorder.cpp)
target_link_libraries(area_recorder_node
  ${catkin_LIBRARIES}
  yaml-cpp
)