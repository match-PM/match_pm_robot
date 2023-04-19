cmake_minimum_required(VERSION 3.8)
project(pm_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pm_client REQUIRED)

add_executable(
  controller
  src/main.cpp
)
set_property(TARGET controller PROPERTY CMAKE_CXX_STANDARD 20)
ament_target_dependencies(controller rclcpp std_msgs sensor_msgs pm_client)

install(TARGETS controller DESTINATION lib/${PROJECT_NAME})

ament_package()
