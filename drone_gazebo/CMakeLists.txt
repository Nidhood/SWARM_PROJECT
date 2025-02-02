cmake_minimum_required(VERSION 3.8)
project(drone_gazebo)

# Explicitly set install prefix
set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/../../install/drone_gazebo CACHE PATH "Install prefix" FORCE)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# install directories
install(DIRECTORY launch models worlds urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()