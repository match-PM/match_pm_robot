cmake_minimum_required(VERSION 3.8)
project(pm_client)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(open62541 REQUIRED)

add_library(
  pm_client SHARED
  src/client.cpp
  src/aerotech_axis.cpp
  src/camera.cpp
  src/pneumatic_cylinder.cpp
  include/pm_client/util.hpp
  include/pm_client/client.hpp
  include/pm_client/robot.hpp
  include/pm_client/aerotech_axis.hpp
  include/pm_client/camera.hpp
  include/pm_client/pneumatic_cylinder.hpp
)
target_include_directories(pm_client PRIVATE include/)

ament_target_dependencies(pm_client open62541)
ament_export_targets(pm_client HAS_LIBRARY_TARGET)
ament_export_dependencies(open62541)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS pm_client
  EXPORT pm_client
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
