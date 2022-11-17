cmake_minimum_required(VERSION 3.8)
project(duro_ros)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)


link_directories("/usr/local/lib/")
include_directories("/usr/local/include/")

add_executable(duro_node src/duro.cpp)
target_link_libraries(duro_node sbp)
ament_target_dependencies(duro_node rclcpp std_msgs visualization_msgs)

add_executable(cpp_example src/cpp_example.cc)
target_link_libraries(cpp_example sbp)
ament_target_dependencies(cpp_example rclcpp std_msgs visualization_msgs)




# install(
#   DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}/
# )
# 
# install(
#   TARGETS duro_node
#   DESTINATION lib/${PROJECT_NAME}
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

install(
  TARGETS duro_node cpp_example
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

  
ament_package()