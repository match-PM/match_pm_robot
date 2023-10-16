cmake_minimum_required(VERSION 3.8)
project(pm_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pm_client REQUIRED)

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

add_library(
  pm_hardware_interface SHARED
  src/pm_system.cpp
  include/pm_hardware_interface/pm_system.hpp
  include/pm_hardware_interface/axis.hpp
  include/pm_hardware_interface/pneumatic.hpp
  include/pm_hardware_interface/hoenle_uv.hpp
)
target_include_directories(pm_hardware_interface PRIVATE include/)

ament_target_dependencies(
  pm_hardware_interface
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  pm_client
)
pluginlib_export_plugin_description_file(hardware_interface pm_hardware_interface.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS pm_hardware_interface
  EXPORT pm_hardware_interface
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(pm_hardware_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  hardware_interface
  pluginlib
  rclcpp_lifecycle
  pm_client
)

ament_package()
