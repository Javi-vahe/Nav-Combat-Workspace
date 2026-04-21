cmake_minimum_required(VERSION 3.0.2)
project(decision_behavior_tree)

# 设置构建类型为Release
set(CMAKE_BUILD_TYPE "Release")

# 添加当前包路径作为宏定义
add_definitions(-DMY_PACKAGE_PATH=\"${PROJECT_SOURCE_DIR}\")

find_package(catkin REQUIRED COMPONENTS
  actionlib
  behaviortree_cpp_v3
  # behaviortree_cpp
  move_base_msgs
  roscpp
  std_msgs
  geometry_msgs
  tf2_ros
)

find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES decision_behavior_tree
#  CATKIN_DEPENDS actionlib behaviortree_cpp_v3 move_base_msgs roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)


add_executable(decision_behavior_tree_node src/main.cpp)
add_dependencies(decision_behavior_tree_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(decision_behavior_tree_node
  ${catkin_LIBRARIES}
  yaml-cpp)