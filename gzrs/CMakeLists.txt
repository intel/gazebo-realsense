cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_library(RealSensePlugin SHARED RealSensePlugin.cc)
target_link_libraries(RealSensePlugin ${GAZEBO_libraries})

install(TARGETS RealSensePlugin DESTINATION ${GZRS_PLUGIN_PATH})

