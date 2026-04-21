cmake_minimum_required(VERSION 3.0.2)
project(clear_costmap_recovery)

# 设置构建类型为Release
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED
        COMPONENTS
            cmake_modules
            costmap_2d
            nav_core
            pluginlib
            roscpp
            tf2_ros
            geometry_msgs
            std_srvs
        )

find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES clear_costmap_recovery
    CATKIN_DEPENDS
        costmap_2d
        nav_core
        pluginlib
        roscpp
        tf2_ros
        geometry_msgs
        std_srvs
)

# 构建 clear_costmap_recovery 库
add_library(clear_costmap_recovery src/clear_costmap_recovery.cpp)
add_dependencies(clear_costmap_recovery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(clear_costmap_recovery ${catkin_LIBRARIES})

# 构建 costmap_cleaner 可执行文件
add_executable(costmap_cleaner src/costmap_cleaner.cpp)
add_dependencies(costmap_cleaner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(costmap_cleaner ${catkin_LIBRARIES})

## 配置测试
if(CATKIN_ENABLE_TESTING)
  # Find package test dependencies
  find_package(rostest REQUIRED)

  # Add the test folder to the include directories
  include_directories(test)

  # Create targets for test executables
  add_rostest_gtest(clear_tester test/clear_tests.launch test/clear_tester.cpp)
  target_link_libraries(clear_tester clear_costmap_recovery ${GTEST_LIBRARIES})
endif()

# 安装 clear_costmap_recovery 库
install(TARGETS clear_costmap_recovery
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

# 安装 costmap_cleaner 可执行文件
install(TARGETS costmap_cleaner
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

# 安装插件描述文件
install(FILES ccr_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装头文件
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
