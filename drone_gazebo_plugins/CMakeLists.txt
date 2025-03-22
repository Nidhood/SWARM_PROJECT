cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(drone_gazebo_plugins)

# Agregar directorios de módulos personalizados (para encontrar FindMAVLink.cmake)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Encontrar dependencias de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(service_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(type_description_interfaces REQUIRED)

# Encontrar dependencias de Gazebo Harmonic
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)


# Find MAVLink
find_package(MAVLink)

# Encontrar Protobuf
find_package(Protobuf REQUIRED)
set(Protobuf_IMPORT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/msgs" ${Protobuf_IMPORT_DIRS})

# Encontrar dependencias de gz-msgs11 y agregar su ruta para que Protobuf encuentre vector3d.proto
find_package(gz-msgs11 REQUIRED)
include_directories(${gz-msgs11_INCLUDE_DIRS})
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
)

# Incluir directorios locales y headers propios del paquete
include_directories(
  include/drone_gazebo_plugins
  ${CMAKE_BINARY_DIR}
  ${Boost_INCLUDE_DIRS}
)

# Agregar la biblioteca principal (conecta con PX4/MAVLink SITL)
add_library(drone_gazebo_plugins SHARED 
  src/gazebo_mavlink_interface.cpp 
  src/mavlink_interface.cpp)
set_property(TARGET drone_gazebo_plugins PROPERTY CXX_STANDARD 17)
target_link_libraries(drone_gazebo_plugins
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gazebo_mavlink_sensor_msgs
)

# Generar los archivos de protos para mensajes de sensor y para imu.proto
PROTOBUF_GENERATE_CPP(SEN_PROTO_SRCS SEN_PROTO_HDRS
  msgs/Pressure.proto
  msgs/MagneticField.proto
  msgs/Groundtruth.proto
  msgs/SITLGps.proto
  /usr/share/gz/gz-msgs11/protos/gz/msgs/imu.proto
)
set_source_files_properties(${SEN_PROTO_SRCS} ${SEN_PROTO_HDRS} PROPERTIES GENERATED TRUE)
add_library(gazebo_mavlink_sensor_msgs SHARED ${SEN_PROTO_SRCS})
target_link_libraries(gazebo_mavlink_sensor_msgs PRIVATE gz-msgs11::gz-msgs11)

# Biblioteca para el plugin de barómetro
add_library(gazebo_barometer_plugin SHARED src/gazebo_barometer_plugin.cpp)
set_property(TARGET gazebo_barometer_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_barometer_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gazebo_mavlink_sensor_msgs
)

# Biblioteca para el plugin de magnetómetro
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

# Biblioteca para el plugin de GPS
add_library(gazebo_gps_plugin SHARED src/gazebo_gps_plugin.cpp)
set_property(TARGET gazebo_gps_plugin PROPERTY CXX_STANDARD 17)
target_link_libraries(gazebo_gps_plugin
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gazebo_mavlink_sensor_msgs
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
