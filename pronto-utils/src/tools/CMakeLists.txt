################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(PkgConfig REQUIRED)

pkg_check_modules(PCL_IO pcl_io-1.7)
if(PCL_IO_FOUND)
  message("---- PCL IO was found (in pronto_vis)\n")
  set(PCL_IO_PKG_CONFIG "pcl_io-1.7")
  set(USE_PRONTO_VIS_PCL TRUE)
endif()

if(NOT PCL_IO_FOUND)
  message("---- PCL IO not found in tools\n")
  message("---- PCL IO not found. Not building tools\n")
  return()
endif()


################################################################
message(STATUS "PCL_IO_PKG_CONFIG={${PCL_IO_PKG_CONFIG}}")



set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable ")

add_executable(pronto-multisense-to-frame pronto_multisense_to_frame.cpp)
target_link_libraries(pronto-multisense-to-frame boost_system)
pods_use_pkg_config_packages(pronto-multisense-to-frame lcm 
  pronto_vis pronto_lcm)
pods_install_executables(pronto-multisense-to-frame)


##################################################
pods_install_headers(conversions_lcm.hpp point_types.hpp DESTINATION pronto_utils)

set(PCL_LIBRARIES  pcl_io-1.7 pcl_filters-1.7)

##############################################
add_executable (se-pointcloud2-tool pointcloud2_tool.cpp)
target_link_libraries(se-pointcloud2-tool boost_system)
pods_use_pkg_config_packages(se-pointcloud2-tool eigen3
  pronto_vis laser-util
  lcm  ${PCL_LIBRARIES})
pods_install_executables(se-pointcloud2-tool)
