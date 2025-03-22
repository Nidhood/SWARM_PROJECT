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
find_package(type_description_interfaces REQUIRED)

# Find Gazebo dependencies
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs11 REQUIRED)

# Find Protobuf
find_package(Protobuf REQUIRED)

# Find sdformat14
find_package(sdformat14 REQUIRED)

# Find MAVLink using our custom module
find_package(MAVLink REQUIRED)
include_directories(${MAVLINK_INCLUDE_DIRS})

# Find the Protobuf package and add the path to the vector3d.proto file
list(APPEND Protobuf_IMPORT_DIRS "/usr/share/gz/gz-msgs11/protos")


# Incluir directorios de Gazebo Harmonic
include_directories(
  ${gz-sim8_INCLUDE_DIRS}
  ${gz-common5_INCLUDE_DIRS}
  ${gz-math8_INCLUDE_DIRS}
  ${gz-plugin2_INCLUDE_DIRS}
  ${gz-transport13_INCLUDE_DIRS}
)

# Incluir directorios de ROS 2
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${trajectory_msgs_INCLUDE_DIRS}
  ${service_msgs_INCLUDE_DIRS}
  ${type_description_interfaces_INCLUDE_DIRS}
)

# Incluir directorios locales y los headers propios del paquete
include_directories(
  include/drone_gazebo_plugins
  ${CMAKE_BINARY_DIR}
  ${Boost_INCLUDE_DIRS}
)

# Add the library for the gazebo_mavlink_interface
add_library(drone_gazebo_plugins SHARED 
  src/gazebo_mavlink_interface.cpp 
  src/mavlink_interface.cpp)
set_property(TARGET drone_gazebo_plugins PROPERTY CXX_STANDARD 17)
target_link_libraries(drone_gazebo_plugins
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gazebo_mavlink_sensor_msgs
  PRIVATE sdformat14::sdformat14
)

# Generar los archivos de protos para los mensajes de sensor y para imu.proto
PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS
  msgs/Pressure.proto
  msgs/MagneticField.proto
  msgs/Groundtruth.proto
  msgs/SITLGps.proto
)
set_source_files_properties(${SEN_PROTO_SRCS} ${SEN_PROTO_HDRS} PROPERTIES GENERATED TRUE)
add_library(gazebo_mavlink_sensor_msgs SHARED ${SEN_PROTO_SRCS})
target_link_libraries(gazebo_mavlink_sensor_msgs PRIVATE gz-msgs11::gz-msgs11)

# Add the library for the barometer plugin
add_library(gazebo_barometer_plugin SHARED src/gazebo_barometer_plugin.cpp)
set_property(TARGET gazebo_barometer_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_barometer_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gazebo_mavlink_sensor_msgs
)

# Add the library for the magnetometer plugin
add_library(gazebo_magnetometer_plugin SHARED 
  src/gazebo_magnetometer_plugin.cpp 
  src/geo_mag_declination.cpp)
set_property(TARGET gazebo_magnetometer_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_magnetometer_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gazebo_mavlink_sensor_msgs
)

# Add the library for the GPS plugin
add_library(gazebo_gps_plugin SHARED src/gazebo_gps_plugin.cpp)
set_property(TARGET gazebo_gps_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_gps_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gazebo_mavlink_sensor_msgs
)

# install directories
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

include(GNUInstallDirs)
install(TARGETS
  drone_gazebo_plugins
  gazebo_barometer_plugin
  gazebo_magnetometer_plugin
  gazebo_gps_plugin
  gazebo_mavlink_sensor_msgs
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set(CPACK_PACKAGE_CONTACT "Auterion")
include(CPack)

ament_export_dependencies(rclcpp std_msgs trajectory_msgs service_msgs type_description_interfaces)
ament_export_include_directories(include)
ament_export_libraries(drone_gazebo_plugins)

ament_package()
