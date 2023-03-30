cmake_minimum_required(VERSION 3.8)
project(pm_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_subdirectory(pm_client/)

add_executable(
  controller
  src/main.cpp
)
set_property(TARGET controller PROPERTY CMAKE_CXX_STANDARD 20)
target_include_directories(controller PRIVATE ${pm_client_INCLUDE_DIRS})
target_link_libraries(controller pm_client)
ament_target_dependencies(controller rclcpp std_msgs sensor_msgs)

install(TARGETS controller DESTINATION lib/${PROJECT_NAME})

ament_package()

# --------

# cmake_minimum_required(VERSION 3.8)
# project(pm_robot_control)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# add_subdirectory(pm_client/)

# add_executable(
#   controller
#   src/main.cpp
# )
# set_property(TARGET controller PROPERTY CXX_STANDARD 20)
# target_include_directories(controller PRIVATE ${pm_client_INCLUDE_DIRS})
# target_link_libraries(controller pm_client)
