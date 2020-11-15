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
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
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
std_srvs
nav2_msgs
nav_msgs
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

add_executable(localization_homing src/localization_homing.cpp)
ament_target_dependencies(localization_homing ${dependencies})

add_executable(simple_goals src/simple_goals.cpp)
ament_target_dependencies(simple_goals ${dependencies})

add_executable(waypoint_follower src/waypoint_follower.cpp)
ament_target_dependencies(waypoint_follower ${dependencies})

add_executable(straight_test src/tests/straight_test.cpp)
ament_target_dependencies(straight_test ${dependencies})

add_executable(turn_test src/tests/turn_test.cpp)
ament_target_dependencies(turn_test ${dependencies})

add_executable(square_test src/tests/square_test.cpp)
ament_target_dependencies(square_test ${dependencies})

add_executable(error_publisher src/error_publisher.cpp)
ament_target_dependencies(error_publisher ${dependencies})

add_executable(cov_publisher src/cov_publisher.cpp)
ament_target_dependencies(cov_publisher ${dependencies})

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
  localization_homing
  simple_goals
  waypoint_follower
  straight_test
  turn_test
  square_test
  error_publisher
  cov_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_package()
