cmake_minimum_required(VERSION 3.0.2)
project(ackermann_recovery)

## 编译选项
add_compile_options(-std=c++14)

## 查找依赖
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_core
  pluginlib
  geometry_msgs
  costmap_2d
  tf
  tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ackermann_recovery
  CATKIN_DEPENDS roscpp nav_core pluginlib geometry_msgs costmap_2d tf
)

## 包含头文件路径
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## 编译插件库
add_library(ackermann_recovery
  src/ackermann_recovery.cpp
)

target_link_libraries(ackermann_recovery
  ${catkin_LIBRARIES}
)

## 插件描述导出
install(TARGETS ackermann_recovery
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES ackermann_recovery_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

## 安装头文件
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)