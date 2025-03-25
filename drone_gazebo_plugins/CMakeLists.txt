cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(drone_gazebo_plugins)

# Define the macro to keep the placeholders of Boost in the global namespace (to avoid the warning)
add_compile_definitions(BOOST_BIND_GLOBAL_PLACEHOLDERS)

# Disable the "address of packed member" warning in GCC/Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wno-address-of-packed-member)
endif()

# Add the custom module path to find FindMAVLink.cmake
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Find ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(service_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(actuator_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(type_description_interfaces REQUIRED)
find_package(drone_msgs REQUIRED)

# Find Gazebo dependencies
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math7 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-cmake3 REQUIRED)

# Find Protobuf
find_package(Protobuf REQUIRED)

# Find sdformat14
find_package(sdformat14 REQUIRED)

# Find MAVLink using our custom module
find_package(MAVLink REQUIRED)
include_directories(${MAVLINK_INCLUDE_DIRS})

# Find the Protobuf package and add the path to the vector3d.proto file
set(Protobuf_IMPORT_DIRS "/usr/share/gz/gz-msgs10/protos")

# Inlude Gazebo Harmonic directories
include_directories(
  ${gz-sim8_INCLUDE_DIRS}
  ${gz-common5_INCLUDE_DIRS}
  ${gz-math7_INCLUDE_DIRS}
  ${gz-plugin2_INCLUDE_DIRS}
  ${gz-transport13_INCLUDE_DIRS}
  ${gz-msgs10_INCLUDE_DIRS}
  ${gz-cmake3_INCLUDE_DIRS}
)

# Include the directories of the ROS 2 dependencies
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${actuator_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${service_msgs_INCLUDE_DIRS}
  ${type_description_interfaces_INCLUDE_DIRS}
  ${sdformat14_INCLUDE_DIRS}
  ${Protobuf_INCLUDE_DIRS}
  ${drone_msgs_INCLUDE_DIRS}
)

# Generate the protobuf files for the sensor messages
PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS
  msgs/Pressure.proto
  msgs/MagneticField.proto
  msgs/Groundtruth.proto
  msgs/SITLGps.proto
)
set_source_files_properties(${SEN_PROTO_SRCS} ${SEN_PROTO_HDRS} PROPERTIES GENERATED TRUE)
add_library(gazebo_mavlink_sensor_msgs SHARED ${SEN_PROTO_SRCS})
target_link_libraries(gazebo_mavlink_sensor_msgs 
  PRIVATE gz-msgs10::gz-msgs10
)

# Include local directories and the headers of the package
include_directories(
  include/drone_gazebo_plugins
  ${CMAKE_BINARY_DIR}
  ${Boost_INCLUDE_DIRS}
  ${MAVLINK_INCLUDE_DIRS}
)

# Add the library for the gazebo_mavlink_interface
add_library(mavlink_sitl_gzsim_gazebo SHARED 
  src/gazebo_mavlink_interface.cc
  src/mavlink_interface.cc)
set_property(TARGET mavlink_sitl_gzsim_gazebo PROPERTY CXX_STANDARD 17)
target_link_libraries(mavlink_sitl_gzsim_gazebo
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gz-msgs10::gz-msgs10
  PRIVATE gazebo_mavlink_sensor_msgs
)

# Force the linker to link the drone_msgs__rosidl_typesupport_cpp library
find_library(DRONE_MSGS_TYPESUPPORT_CPP_LIB
  NAMES drone_msgs__rosidl_typesupport_cpp
  PATHS ${AMENT_PREFIX_PATH}/drone_msgs/lib
        ${CMAKE_INSTALL_PREFIX}/drone_msgs/lib
)
if(NOT DRONE_MSGS_TYPESUPPORT_CPP_LIB)
  message(FATAL_ERROR "The drone_msgs__rosidl_typesupport_cpp library was not found.")
endif()

# Add the library for the barometer plugin
add_library(gazebo_barometer_plugin SHARED src/gazebo_barometer_plugin.cc)
set_property(TARGET gazebo_barometer_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_barometer_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gazebo_mavlink_sensor_msgs
  PRIVATE rclcpp::rclcpp
  PRIVATE ${DRONE_MSGS_TYPESUPPORT_CPP_LIB}
)

# Add the library for the magnetometer plugin
add_library(gazebo_magnetometer_plugin SHARED 
  src/gazebo_magnetometer_plugin.cc 
  src/geo_mag_declination.cc)
set_property(TARGET gazebo_magnetometer_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_magnetometer_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gazebo_mavlink_sensor_msgs
  PRIVATE rclcpp::rclcpp
  PRIVATE ${DRONE_MSGS_TYPESUPPORT_CPP_LIB}
)

# Add the library for the GPS plugin
add_library(gazebo_gps_plugin SHARED src/gazebo_gps_plugin.cc)
set_property(TARGET gazebo_gps_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_gps_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gazebo_mavlink_sensor_msgs
  PRIVATE rclcpp::rclcpp
  PRIVATE ${DRONE_MSGS_TYPESUPPORT_CPP_LIB}
)

# Force the linker to link the actuator_msgs__rosidl_typesupport_cpp library
find_library(ACTUATOR_MSGS_TYPESUPPORT_CPP_LIB
  NAMES actuator_msgs__rosidl_typesupport_cpp
  PATHS ${AMENT_PREFIX_PATH}/actuator_msgs/lib
        ${CMAKE_INSTALL_PREFIX}/actuator_msgs/lib
)
if(NOT ACTUATOR_MSGS_TYPESUPPORT_CPP_LIB)
  message(FATAL_ERROR "The actuator_msgs__rosidl_typesupport_cpp library was not found.")
endif()

# Add the library for the rotors_command_plugin plugin
add_library(rotors_command_plugin SHARED src/rotors_command_plugin.cc)
set_property(TARGET rotors_command_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(rotors_command_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gz-msgs10::gz-msgs10
  PRIVATE rclcpp::rclcpp
  PRIVATE ${ACTUATOR_MSGS_TYPESUPPORT_CPP_LIB}
)

# Force the linker to link the sensor_msgs__rosidl_typesupport_cpp library
find_library(SENSOR_MSGS_TYPESUPPORT_CPP_LIB
  NAMES sensor_msgs__rosidl_typesupport_cpp
  PATHS ${AMENT_PREFIX_PATH}/sensor_msgs/lib
        ${CMAKE_INSTALL_PREFIX}/sensor_msgs/lib
)
if(NOT SENSOR_MSGS_TYPESUPPORT_CPP_LIB)
  message(FATAL_ERROR "The sensor_msgs__rosidl_typesupport_cpp library was not found.")
endif()

# Add the library for the imu_channel_plugin plugin
add_library(imu_channel_plugin SHARED src/imu_channel_plugin.cc)
set_property(TARGET imu_channel_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(imu_channel_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gz-msgs10::gz-msgs10
  PRIVATE rclcpp::rclcpp
  PRIVATE ${SENSOR_MSGS_TYPESUPPORT_CPP_LIB}
)

# install directories
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# install libraries
include(GNUInstallDirs)
install(TARGETS
  mavlink_sitl_gzsim_gazebo
  gazebo_barometer_plugin
  gazebo_magnetometer_plugin
  gazebo_gps_plugin
  gazebo_mavlink_sensor_msgs
  rotors_command_plugin
  imu_channel_plugin
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set(CPACK_PACKAGE_CONTACT "Auterion")
include(CPack)

ament_export_dependencies(rclcpp std_msgs trajectory_msgs actuator_msgs sensor_msgs service_msgs type_description_interfaces)
ament_export_include_directories(include)
ament_export_libraries(drone_gazebo_plugins)
ament_package()
