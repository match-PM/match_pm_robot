cmake_minimum_required(VERSION 3.8)
project(pm_client)

find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile.in)
  set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

  configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

  add_custom_target(
    doc_doxygen ALL
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
  )
endif(DOXYGEN_FOUND)

set(
  ${PROJECT_NAME}_INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/open62541
  CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE
)

add_library(
  open62541
  open62541/open62541.c
  open62541/open62541.h
)
target_link_libraries(open62541 pthread)

add_library(
  pm_client
  src/client.cpp
  src/aerotech_axis.cpp
)
set_property(TARGET pm_client PROPERTY CXX_STANDARD 20)
set_target_properties(pm_client PROPERTIES LINKER_LANGUAGE CXX)
target_include_directories(pm_client PRIVATE include/)
target_include_directories(pm_client PRIVATE open62541/)
target_link_libraries(pm_client open62541)

