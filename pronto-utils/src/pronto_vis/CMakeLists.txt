################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pronto_vis)

find_package(PkgConfig REQUIRED)

pkg_check_modules(PCL_IO pcl_io-1.7)
if(PCL_IO_FOUND)
  message("---- PCL IO was found (in pronto_vis)\n")
  set(PCL_IO_PKG_CONFIG "pcl_io-1.7")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

pkg_check_modules(PCL_IO_1_8 pcl_io-1.8)
if(PCL_IO_1_8_FOUND)
  message("---- PCL IO_1_8 was found (in pronto_vis)\n")
  set(PCL_IO_PKG_CONFIG "pcl_io-1.8")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

if(NOT PCL_IO_FOUND AND NOT PCL_IO_1_8_FOUND)
  message("---- PCL IO not found in pronto_vis\n")
  #message("---- PCL IO not found. Not building pronto_vis\n")
  #return()
endif()

pkg_check_modules(VISUALIZATION visualization)
if(NOT VISUALIZATION_FOUND)
  message("---- Visualization not found. Not building pronto_vis\n")
  return()
endif()

################################################################
message(STATUS "PCL_IO_PKG_CONFIG={${PCL_IO_PKG_CONFIG}}")
message(STATUS "VISUALIZATION_FOUND={${VISUALIZATION_FOUND}}")

configure_file(pronto_vis_config.h.in pronto_vis_config.h)
include_directories("${CMAKE_CURRENT_BINARY_DIR}")

add_library(pronto_vis SHARED pronto_vis.cpp)
pods_use_pkg_config_packages(pronto_vis ${PCL_IO_PKG_CONFIG} lcm eigen3  
	glib-2.0)

set_target_properties(pronto_vis PROPERTIES SOVERSION 1)
pods_install_libraries(pronto_vis)
pods_install_headers(pronto_vis.hpp ${CMAKE_CURRENT_BINARY_DIR}/pronto_vis_config.h DESTINATION pronto_utils)
pods_install_pkg_config_file(pronto_vis
    LIBS -lpronto_vis 
    REQUIRES ${PCL_IO_PKG_CONFIG} lcmtypes_visualization pronto_math eigen3
    VERSION 0.0.1)

#    REQUIRES pcl_io-1.7 lcmtypes_visualization pronto_math

