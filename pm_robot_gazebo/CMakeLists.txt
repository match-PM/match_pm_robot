cmake_minimum_required(VERSION 3.8)
project(pm_robot_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rmw REQUIRED)
# find_package(std_msgs REQUIRED)
find_package(rmw_fastrtps_cpp REQUIRED)
# Find Gazebo
find_package(gazebo REQUIRED)

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

install(
  DIRECTORY world src
  DESTINATION share/${PROJECT_NAME}
)



#for gazebo plugins
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
)            
link_directories(${GAZEBO_LIBRARY_DIRS})               
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")




# for compiling plugin
add_library(light_control_plugin SHARED src/light_control_plugin.cc)
target_link_libraries(light_control_plugin ${GAZEBO_LIBRARIES})

target_include_directories(light_control_plugin
  PUBLIC
  ${rclcpp_INCLUDE_DIRS}
)

target_link_libraries(light_control_plugin
  ${rclcpp_LIBRARIES}
)


ament_package()
