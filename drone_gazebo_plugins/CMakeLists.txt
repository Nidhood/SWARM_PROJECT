cmake_minimum_required(VERSION 3.8)
project(drone_gazebo_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Find Gazebo Harmonic dependencies
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)

# include directories
include_directories(
  include/drone_gazebo_plugins
  ${rclcpp_INCLUDE_DIRS} 
  ${std_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${gz-sim8_INCLUDE_DIRS}  
  ${gz-common5_INCLUDE_DIRS} 
  ${gz-math8_INCLUDE_DIRS} 
  ${gz-plugin2_INCLUDE_DIRS} 
  ${gz-transport13_INCLUDE_DIRS}
)

# Add the library
add_library(${PROJECT_NAME} SHARED src/channel_plugin.cc)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

# Link Gazebo Harmonic libraries
target_link_libraries(${PROJECT_NAME}
  gz-sim8::gz-sim8
  gz-common5::gz-common5
  gz-math7::gz-math7
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
)

# Link ROS 2 libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  trajectory_msgs
)

# Install the library
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)

# Install the headers
install(DIRECTORY include/
  DESTINATION include
)

# install directories
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# Export dependencies
ament_export_dependencies(rclcpp std_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Enable testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()
