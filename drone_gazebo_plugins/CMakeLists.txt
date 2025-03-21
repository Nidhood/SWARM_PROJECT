cmake_minimum_required(VERSION 3.8)
project(drone_gazebo_plugins)

# Agregar directorios de módulos personalizados (para encontrar FindMAVLink.cmake)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# Encontrar dependencias de ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Encontrar dependencias de Gazebo Harmonic
find_package(gz-sim8 REQUIRED)
find_package(gz-common5 REQUIRED)
find_package(gz-math8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)

# Encontrar Protobuf
find_package(Protobuf REQUIRED)

# Encontrar MAVLink usando nuestro módulo personalizado
find_package(MAVLink REQUIRED)

# Incluir directorios de cabeceras de MAVLink (definido en FindMAVLink.cmake)
include_directories(${MAVLINK_INCLUDE_DIRS})

# Configurar Protobuf para que busque los .proto en la carpeta msgs de tu proyecto y en la ruta donde se encuentra vector3d.proto
# En este ejemplo, se asume que vector3d.proto se ha copiado o se encuentra en msgs/
set(Protobuf_IMPORT_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/msgs" ${Protobuf_IMPORT_DIRS})

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


# Agregar la biblioteca principal (puedes ajustar el nombre o los archivos fuente según corresponda)
add_library(${PROJECT_NAME} SHARED src/channel_plugin.cc)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

# Enlazar las bibliotecas de Gazebo Harmonic
target_link_libraries(${PROJECT_NAME}
  gz-sim8::gz-sim8
  gz-common5::gz-common5
  gz-math8::gz-math8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
)

# Enlazar dependencias ROS 2
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  trajectory_msgs
)

# (Opcional) Si tu paquete utiliza generación de protos, agrégalo aquí
# Por ejemplo, si tienes protos en la carpeta msgs:
# PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS msgs/Pressure.proto msgs/MagneticField.proto)
# add_library(mavlink_sensor_msgs SHARED ${PROTO_SRCS})
# target_link_libraries(mavlink_sensor_msgs PRIVATE ${Protobuf_LIBRARIES})

# Instalar la biblioteca
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib
)

# Instalar los headers
install(DIRECTORY include/
  DESTINATION include
)

# Instalar directorios adicionales (por ejemplo, urdf y msgs)
install(DIRECTORY urdf msgs
  DESTINATION share/${PROJECT_NAME}
)

# Exportar dependencias
ament_export_dependencies(rclcpp std_msgs)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

# Habilitar testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
