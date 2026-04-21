cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  rospy
  dynamic_reconfigure
)

find_package(orocos_kdl REQUIRED)

############################
## dynamic reconfiure api ##
############################
generate_dynamic_reconfigure_options(
  cfg/PurePursuit.cfg
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs tf2 tf2_ros rospy dynamic_reconfigure
  )


###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(pure_pursuit src/pure_pursuit.cpp)
target_link_libraries(pure_pursuit ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(pure_pursuit ${PROJECT_NAME}_gencfg)

add_executable(path_publisher src/path_publisher.cpp)
target_link_libraries(path_publisher ${catkin_LIBRARIES})