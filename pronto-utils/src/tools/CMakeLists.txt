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
