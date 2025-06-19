cmake_minimum_required(VERSION 3.8)
project(pm_opcua_skills_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pm_opcua_skills_msgs REQUIRED)
find_package(pm_client REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pm_opcua_skills_msgs REQUIRED)
find_package(pm_robot_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(pm_msgs REQUIRED)

include_directories(${CMAKE_INSTALL_PREFIX}/include)



add_executable(
  controller
  src/controller.cpp
)

ament_target_dependencies(
  controller
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  pm_opcua_skills_msgs
  pm_client
)

install(TARGETS
  controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
