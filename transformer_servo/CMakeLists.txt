cmake_minimum_required(VERSION 3.0.2)
project(transformer_servo)
# 设置构建类型为Release
set(CMAKE_BUILD_TYPE "Release")

# 设置 C++ 标准为 C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)  # 确保编译器必须支持 C++20

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  tf
  tf2_ros
  dynamic_reconfigure
  zllc_msgs
  laser_geometry
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES transformer_zllc
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(cmd_vel_to_angle_node src/cmd_vel_to_angle.cpp)
target_link_libraries(cmd_vel_to_angle_node ${catkin_LIBRARIES})

add_executable(angular_to_angle_node src/angular_to_angle.cpp)
target_link_libraries(angular_to_angle_node ${catkin_LIBRARIES})

add_executable(pub_robot_state_node src/pub_robot_state.cpp)
target_link_libraries(pub_robot_state_node ${catkin_LIBRARIES})

add_executable(teb_tolerance_changer src/teb_tolerance_changer.cpp)
target_link_libraries(teb_tolerance_changer ${catkin_LIBRARIES})

add_executable(gimbal_tf_node src/gimbal_tf.cpp)
target_link_libraries(gimbal_tf_node ${catkin_LIBRARIES})

add_executable(material_tf_puber src/material_tf_puber.cpp)
target_link_libraries(material_tf_puber ${catkin_LIBRARIES})

add_executable(scan_to_pcl2 src/scan_to_pcl2.cpp)
target_link_libraries(scan_to_pcl2 ${catkin_LIBRARIES})

add_executable(material_processor src/material_processor.cpp)
target_link_libraries(material_processor ${catkin_LIBRARIES})

add_executable(control_gimbal_to_material src/control_gimbal_to_material.cpp)
target_link_libraries(control_gimbal_to_material ${catkin_LIBRARIES})

add_executable(bond src/bond.cpp)
target_link_libraries(bond ${catkin_LIBRARIES})