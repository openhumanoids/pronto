################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pronto_vis)

find_package(PkgConfig REQUIRED)

pkg_check_modules(PCL_IO pcl_io-1.7)
if(NOT PCL_IO_FOUND)
  message("---- PCL IO not found. Not building pronto_vis\n")
  return()
endif()

pkg_check_modules(VISUALIZATION visualization)
if(NOT VISUALIZATION_FOUND)
  message("---- Visualization not found. Not building pronto_vis\n")
  return()
endif()


add_library(pronto_vis SHARED pronto_vis.cpp)

pods_use_pkg_config_packages(pronto_vis pcl_io-1.7 lcm eigen3   bot2-core  
	glib-2.0)
#  lcmtypes_pronto-tools )

# set the library API version.  Increment this every time the public API changes.
set_target_properties(pronto_vis PROPERTIES SOVERSION 1)
pods_install_libraries(pronto_vis)
pods_install_headers(pronto_vis.hpp DESTINATION pronto_utils)
pods_install_pkg_config_file(pronto_vis
    LIBS -lpronto_vis 
    REQUIRES bot2-vis pcl_io-1.7 lcmtypes_visualization pronto_math
    VERSION 0.0.1)

