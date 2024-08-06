cmake_minimum_required(VERSION 3.8)
project(pm_moveit_server)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(urdfdom REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pm_moveit_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

# uncomment the following section in order to fill in further dependencies manually.
# find_package(<dependency> REQUIRED)

# Include Cpp "include" directory
include_directories(
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_msgs_INCLUDE_DIRS}
  ${moveit_INCLUDE_DIRS}
  ${urdf_INCLUDE_DIRS}  
  ${urdfdom_INCLUDE_DIRS}  
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${shapes_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${graph_msgs_INCLUDE_DIRS}
  include
)

add_executable(pm_moveit_server src/pm_moveit_server.cpp)

ament_target_dependencies(pm_moveit_server
  rclcpp 
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  ${THIS_PACKAGE_INCLUDE_DEPENDS} 
  moveit_msgs
  tf2_ros
  std_msgs
  graph_msgs
  moveit_visual_tools
  tf2
  pm_moveit_interfaces
)

# Install Cpp executables
install(TARGETS
pm_moveit_server
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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

ament_package()
