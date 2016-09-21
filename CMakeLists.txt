cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Find Gazebo and Includes
find_package(gazebo REQUIRED)

# Custom instalation paths
if (GZRS_PLUGIN_INSTALL_PATH)
    GET_FILENAME_COMPONENT(GZRS_PLUGIN_PATH ${GZRS_PLUGIN_INSTALL_PATH} ABSOLUTE)
else()
    set(GZRS_PLUGIN_PATH ${GAZEBO_PLUGIN_PATH})
endif()

# Gazebo 7 does not provide a ${GAZEBO_MODEL_PATH} variable.
# That's why we are accessing it relatively to ${GAZEBO_MEDIA_PATH}.
# A proper ${GAZEBO_MODEL_PATH} variable might be available in Gazebo 8+.
if (GZRS_MODEL_INSTALL_PATH)
    GET_FILENAME_COMPONENT(GZRS_MODEL_PATH ${GZRS_MODEL_INSTALL_PATH} ABSOLUTE)
elseif(GAZEBO_MODEL_PATH)
    set(GZRS_MODEL_PATH ${GAZEBO_MODEL_PATH})
else()
    GET_FILENAME_COMPONENT(GZRS_MODEL_PATH ${GAZEBO_MEDIA_PATH}/../models ABSOLUTE)
endif()

# Compiler flags
set(CUSTOM_COMPILE_FLAGS "-g -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} ${CUSTOM_COMPILE_FLAGS}")

# Include and Link directories
include_directories( ${GAZEBO_INCLUDE_DIRS} )
link_directories( ${GAZEBO_LIBRARY_DIRS} )

add_subdirectory(gzrs)
add_subdirectory(models)

