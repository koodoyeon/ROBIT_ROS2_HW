cmake_minimum_required(VERSION 3.5)
project(turtlesim_draw)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(turtlesim_draw src/turtlesim_draw.cpp)
ament_target_dependencies(turtlesim_draw rclcpp turtlesim geometry_msgs)

install(TARGETS
  turtlesim_draw
  DESTINATION lib/${PROJECT_NAME})

ament_package()

