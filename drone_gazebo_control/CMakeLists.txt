cmake_minimum_required(VERSION 3.8)
project(drone_gazebo_control)

# Default to C11
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# include directories
include_directories(include/drone_gazebo_control)

# install directories
install(DIRECTORY urdf config
  DESTINATION share/${PROJECT_NAME}
)

# Add the executables
add_executable(drone_test_node src/drone_test.cpp)
ament_target_dependencies(drone_test_node 
  rclcpp 
  rclcpp_action
  control_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install
install(TARGETS
drone_test_node
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
