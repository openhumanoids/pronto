################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pronto_lcm)

set(PCL_LIBRARIES  pcl_io-1.7)

add_library(pronto_lcm SHARED pronto_lcm.cpp)

pods_use_pkg_config_packages(pronto_lcm ${PCL_LIBRARIES} lcm eigen3
   bot2-core  glib-2.0   opencv
   )
#kinect-utils lcmtypes_kinect  lcmtypes_multisense)

# set the library API version.  Increment this every time the public API changes.
set_target_properties(pronto_lcm PROPERTIES SOVERSION 1)
pods_install_libraries(pronto_lcm)
pods_install_headers(pronto_lcm.hpp DESTINATION pronto_utils)

target_link_libraries(pronto_lcm
    z)
pods_install_pkg_config_file(pronto_lcm
    LIBS -lpronto_lcm -lz
    REQUIRES bot2-vis  image-utils eigen3 pcl_io-1.7 flann
    VERSION 0.0.1)
# kinect-utils

