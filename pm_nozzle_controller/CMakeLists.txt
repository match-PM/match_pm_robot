cmake_minimum_required(VERSION 3.8)
project(pm_nozzle_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(pm_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

generate_parameter_library(
  pm_nozzle_controller_parameters
  src/pm_nozzle_controller_parameters.yaml
)

add_library(
  pm_nozzle_controller SHARED
  src/nozzle_controller.cpp
  include/pm_nozzle_controller/nozzle_controller.hpp
)
target_include_directories(pm_nozzle_controller PRIVATE include/)
target_link_libraries(pm_nozzle_controller pm_nozzle_controller_parameters)

ament_target_dependencies(
  pm_nozzle_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  pm_msgs
)
pluginlib_export_plugin_description_file(controller_interface pm_nozzle_controller.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS pm_nozzle_controller pm_nozzle_controller_parameters
  EXPORT pm_nozzle_controller 
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(pm_nozzle_controller HAS_LIBRARY_TARGET)
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  pm_pneumatic_controller_parameters
)

ament_package()
