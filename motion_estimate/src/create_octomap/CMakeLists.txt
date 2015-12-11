set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(create_octomap)


find_package(PkgConfig REQUIRED)

pkg_check_modules(PCL_IO pcl_io-1.7)
if(PCL_IO_FOUND)
  message("---- PCL IO was found (in create_octomap)\n")
endif()

if(NOT PCL_IO_FOUND)
  message("---- PCL IO not found in create_octomap\n")
  message("---- PCL IO not found. Not building create_octomap\n")
  return()
endif()

message(STATUS "PCL_IO_PKG_CONFIG={${PCL_IO_PKG_CONFIG}}")


################
add_executable(se-create-octomap create_octomap.cpp convert_octomap.cpp cloud_accumulate.cpp)
pods_use_pkg_config_packages(se-create-octomap  
    path-util ${REQUIRED_PACKAGES}
    pronto_vis  octomap eigen3
    octomap-util)
#    drc-utils
target_link_libraries (se-create-octomap lcm pronto_math 
                        boost_system boost_thread)
pods_install_executables(se-create-octomap )
