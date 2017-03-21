################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pronto_lidar_filters)

###############
add_library(pronto_lidar_filters SHARED lidar_filters.cpp)
set_target_properties(pronto_lidar_filters PROPERTIES SOVERSION 1)
pods_install_libraries(pronto_lidar_filters)
pods_install_headers( lidar_filters.hpp DESTINATION pronto_lidar_filters)
pods_use_pkg_config_packages(pronto_lidar_filters lcm)