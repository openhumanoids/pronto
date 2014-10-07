set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable ")




add_executable(test-point-cloud test_point_cloud.cpp)
target_link_libraries(test-point-cloud boost_system)
pods_use_pkg_config_packages(test-point-cloud lcm 
  pronto_vis pronto_lcm)
pods_install_executables(test-point-cloud)


pkg_check_modules(PCL_IO pcl_io-1.7)
if(NOT PCL_IO_FOUND)
  message("---- PCL IO not found. Not building pcl-dependent test modules\n")
  return()
endif()


add_executable(test-point-cloud test_point_cloud_pcl.cpp)
target_link_libraries(test-point-cloud boost_system)
pods_use_pkg_config_packages(test-point-cloud lcm 
  pronto_vis pronto_lcm)
pods_install_executables(test-point-cloud)
