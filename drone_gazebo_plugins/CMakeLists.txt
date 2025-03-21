cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(mavlink_sitl_ign_gazebo)

# Agregar directorios de módulos si tienes cmake personalizados
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

# Encontrar Boost
find_package(Boost 1.58 REQUIRED COMPONENTS system thread filesystem)

# Dependencias de Gazebo Harmonic
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport14 REQUIRED)
find_package(gz-msgs11 REQUIRED)

# Incluir la ruta de los headers de gz-msgs11 para que protoc encuentre vector3d.proto
include_directories(${gz-msgs11_INCLUDE_DIRS})

# Forzar que Protobuf busque en la ruta donde se encuentra vector3d.proto
list(APPEND Protobuf_IMPORT_DIRS "/usr/share/gz/gz-msgs11/protos")

# Incluir la ruta de MAVLink (header-only)
include_directories("$ENV{HOME}/Documents/GitHub/mavlink/install/include")

# Encontrar Protobuf
find_package(Protobuf REQUIRED)

# Configurar Protobuf para que busque los .proto en la carpeta msgs de tu proyecto (si la usas)
set(Protobuf_IMPORT_DIRS ${Protobuf_IMPORT_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/msgs")

# Generar los archivos de protos para los mensajes de sensor y para imu.proto
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

# Incluir directorios adicionales
include_directories(${CMAKE_BINARY_DIR})
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)
# Agregar la carpeta que contiene tus headers propios
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/drone_gazebo_plugins)

# Biblioteca para la interfaz MAVLink SITL (plugin principal)
add_library(mavlink_sitl_ign_gazebo SHARED 
  src/gazebo_mavlink_interface.cpp 
  src/mavlink_interface.cpp)
set_property(TARGET mavlink_sitl_ign_gazebo PROPERTY CXX_STANDARD 17)
target_link_libraries(mavlink_sitl_ign_gazebo
  PRIVATE ${Boost_LIBRARIES}
  PRIVATE gz-plugin2::gz-plugin2
  PRIVATE gz-sim8::gz-sim8
  PRIVATE gazebo_mavlink_sensor_msgs
)

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
  mavlink_sitl_ign_gazebo
  gazebo_barometer_plugin
  gazebo_magnetometer_plugin
  gazebo_gps_plugin
  gazebo_mavlink_sensor_msgs
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
)

set(CPACK_PACKAGE_CONTACT "Auterion")
include(CPack)
