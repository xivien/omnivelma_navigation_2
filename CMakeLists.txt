cmake_minimum_required(VERSION 3.5)
project(omnivelma_navigation_2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)


set(dependencies
rclcpp
sensor_msgs
std_msgs
PCL
pcl_conversions
tf2
tf2_geometry_msgs
tf2_ros
laser_geometry
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(laserscan_multi_merger src/laserscan_multi_merger.cpp)
ament_target_dependencies(laserscan_multi_merger ${dependencies})

# Install launch files.
install(DIRECTORY
  launch
  maps
  params
  rviz2
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS
  laserscan_multi_merger
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_package()
