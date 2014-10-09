################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pronto_lcm)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PCL_IO pcl_io-1.7)
if(PCL_IO_FOUND)
  message("---- PCL IO was found (in pronto_lcm)\n")
  set(PCL_IO_PKG_CONFIG "pcl_io-1.7")
  set(USE_PCL TRUE)
endif()
if(NOT PCL_IO_FOUND)
  message("---- PCL IO not found in pronto_lcm\n")
  #message("---- PCL IO not found. Not building pronto_vis\n")
  #return()
endif()


configure_file(pronto_lcm_config.h.in pronto_lcm_config.h)
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

add_library(pronto_lcm SHARED pronto_lcm.cpp)

pods_use_pkg_config_packages(pronto_lcm ${PCL_IO_PKG_CONFIG} lcm eigen3
   bot2-core  glib-2.0   opencv
   )

set_target_properties(pronto_lcm PROPERTIES SOVERSION 1)
pods_install_libraries(pronto_lcm)
pods_install_headers(pronto_lcm.hpp ${CMAKE_CURRENT_BINARY_DIR}/pronto_lcm_config.h DESTINATION pronto_utils)

target_link_libraries(pronto_lcm
    z)
pods_install_pkg_config_file(pronto_lcm
    LIBS -lpronto_lcm -lz
    REQUIRES bot2-vis  image-utils eigen3 ${PCL_IO_PKG_CONFIG} flann
    VERSION 0.0.1)
# kinect-utils

