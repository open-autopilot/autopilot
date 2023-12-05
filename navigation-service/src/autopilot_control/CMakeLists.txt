cmake_minimum_required(VERSION 3.5)
project(autopilot_control)

#Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

 
# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
 
install(
  DIRECTORY config scripts
  DESTINATION share/${PROJECT_NAME}
)
 
# Install Python modules
ament_python_install_package(scripts)
 
# Install Python executables
install(PROGRAMS
  scripts/logged_waypoint_follower.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()