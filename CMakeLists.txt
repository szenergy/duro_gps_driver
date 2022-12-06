cmake_minimum_required(VERSION 3.5)
project(duro_gps_driver)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
# add_compile_options(-std=c++17 -O2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include 
  "/usr/local/include"
)

set(ament_dependencies
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs
)
link_directories("/usr/local/lib/")
include_directories("/usr/local/include/")

add_executable(duro_node src/duro_node.cpp src/utm.cpp src/fake_orientation.cpp)

ament_target_dependencies(duro_node ${ament_dependencies})
target_link_libraries(duro_node sbp)

## Install
## Mark executables and/or libraries for installation
install(TARGETS duro_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include/${PROJECT_NAME}
)

## Mark launch files for installation
install(DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

ament_package()