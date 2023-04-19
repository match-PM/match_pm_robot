cmake_minimum_required(VERSION 3.8)
project(open62541)

find_package(ament_cmake REQUIRED)

add_library(
  open62541
  src/open62541.c
  include/open62541/open62541.h
)
target_include_directories(open62541 PRIVATE include/open62541)
target_link_libraries(open62541 pthread)

ament_export_targets(open62541 HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/open62541
  DESTINATION include
)

install(
  TARGETS open62541
  EXPORT open62541
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
