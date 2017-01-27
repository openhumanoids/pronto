set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(cloud_accumulate)

################
add_library(cloud_accumulate SHARED cloud_accumulate.cpp)
target_link_libraries(cloud_accumulate boost_system boost_filesystem boost_thread)
set_target_properties(cloud_accumulate PROPERTIES SOVERSION 1)
pods_install_libraries(cloud_accumulate)
pods_install_headers( cloud_accumulate.hpp DESTINATION cloud_accumulate)
pods_use_pkg_config_packages(cloud_accumulate eigen3 pronto_vis laser-util
  lcm bot2-frames ${REQUIRED_PACKAGES})
pods_install_pkg_config_file(cloud_accumulate 
  LIBS 
  REQUIRES eigen3 
  VERSION 0.0.1)