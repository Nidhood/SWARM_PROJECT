cmake_minimum_required(VERSION 3.8)
project(drone_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

# install each launch file
install(DIRECTORY launch rviz urdf meshes worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
