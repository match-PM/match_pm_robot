cmake_minimum_required(VERSION 3.16)
project(opcua_server)

# set(CMAKE_C_STANDARD 99)
# set(CMAKE_CXX_STANDARD 98)

find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)

add_library(open62541 open62541/open62541.c open62541/open62541.h)

add_executable(opcua_server  src/main.cpp)
set_property(TARGET opcua_server PROPERTY CMAKE_CXX_STANDARD 20)
target_compile_options(opcua_server PRIVATE -Wall -Wextra -Wpedantic)
target_include_directories(opcua_server PRIVATE open62541/)
# target_precompile_headers(opcua_server PRIVATE src/open62541.h)
target_link_libraries(opcua_server open62541 pthread)
# ament_target_dependencies(opcua_server rclcpp)
install(TARGETS opcua_server DESTINATION lib/${PROJECT_NAME})

ament_package()

# -------

# cmake_minimum_required(VERSION 3.16)
# project(opcua_server)

# add_library(open62541 open62541/open62541.c open62541/open62541.h)

# add_executable(opcua_server  src/main.cpp)
# set_property(TARGET opcua_server PROPERTY CMAKE_CXX_STANDARD 20)
# target_compile_options(opcua_server PRIVATE -Wall -Wextra -Wpedantic)
# target_include_directories(opcua_server PRIVATE open62541/)
# target_link_libraries(opcua_server open62541 pthread)
