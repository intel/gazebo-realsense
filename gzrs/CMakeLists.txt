cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo and Includes
find_package(gazebo REQUIRED)
include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )
set(CUSTOM_COMPILE_FLAGS "-g -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} ${CUSTOM_COMPILE_FLAGS}")

# Build Plugin
add_library(gzrs_plugin SHARED gzrs_plugin.cc)
target_link_libraries(gzrs_plugin ${GAZEBO_libraries})
