cmake_minimum_required(VERSION 3.8)
project(dynamic_logistics_warehouse)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)

install(
  DIRECTORY
    launch
    models
    worlds
    maps
    rviz
    config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
