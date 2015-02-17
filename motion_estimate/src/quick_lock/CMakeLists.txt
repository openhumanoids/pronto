set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(quick_octomap)


################
add_executable(se-quick-lock quick_lock.cpp)
pods_use_pkg_config_packages(se-quick-lock
    path-util ${REQUIRED_PACKAGES}
    pronto_vis)
target_link_libraries (se-quick-lock lcm pronto_math
                        boost_system boost_thread)
pods_install_executables(se-quick-lock )


add_executable(se-pose-compare pose_compare.cpp)
pods_use_pkg_config_packages(se-pose-compare
    path-util ${REQUIRED_PACKAGES}
    pronto_vis)
target_link_libraries (se-pose-compare lcm pronto_math
                        boost_system boost_thread)
pods_install_executables(se-pose-compare )



add_executable(se-yaw-lock yaw_lock.cpp)
pods_use_pkg_config_packages(se-yaw-lock
    path-util ${REQUIRED_PACKAGES}
    pronto_vis)
target_link_libraries (se-yaw-lock lcm pronto_math
                        boost_system boost_thread)
pods_install_executables(se-yaw-lock )
