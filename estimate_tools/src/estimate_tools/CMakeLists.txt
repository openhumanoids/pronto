cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(estimate_tools)


################


add_library(estimate_tools  SHARED imu_stream.cpp iir_notch.cpp)
pods_use_pkg_config_packages(estimate_tools eigen3 lcmtypes_bot2-core)
set_target_properties(estimate_tools PROPERTIES SOVERSION 1)
pods_install_libraries(estimate_tools)
pods_install_headers( imu_stream.hpp iir_notch.hpp
              DESTINATION estimate_tools)
pods_install_pkg_config_file(estimate_tools
  LIBS -lestimate_tools -llcmtypes_bot2-core
  REQUIRES eigen3
  VERSION 0.0.1)




