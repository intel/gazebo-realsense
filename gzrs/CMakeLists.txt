cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Build Plugin
add_library(RealSensePlugin SHARED RealSensePlugin.cc)
target_link_libraries(RealSensePlugin ${GAZEBO_libraries})

install(TARGETS RealSensePlugin DESTINATION ${GAZEBO_PLUGIN_PATH})
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../models/ DESTINATION $ENV{HOME}/.gazebo/models)

