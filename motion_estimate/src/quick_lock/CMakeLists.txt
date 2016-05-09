set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(yaw_lock)



################
set(REQUIRED_PACKAGES  kdl-parser forward-kinematics)

add_library(yaw_lock  SHARED yaw_lock.cpp)
pods_use_pkg_config_packages(yaw_lock estimate_tools
  ${REQUIRED_PACKAGES} 
  lcm eigen3)
set_target_properties(yaw_lock PROPERTIES SOVERSION 1)
pods_install_libraries(yaw_lock)
pods_install_headers( yaw_lock.hpp
               DESTINATION yaw_lock)
pods_install_pkg_config_file(yaw_lock
  LIBS -lyaw_lock
  VERSION 0.0.1)


################
add_executable(se-yaw-lock yaw_lock_app.cpp)
pods_use_pkg_config_packages(se-yaw-lock yaw_lock
    model-client   bot2-param-client  urdf orocos-kdl kdl-parser forward-kinematics
    path-util ${REQUIRED_PACKAGES}
    pronto_vis)
target_link_libraries (se-yaw-lock lcm pronto_math
                        boost_system boost_thread)
pods_install_executables(se-yaw-lock )


################ quick lock isn't used anymore #####
add_executable(se-quick-lock quick_lock.cpp)
pods_use_pkg_config_packages(se-quick-lock
    path-util ${REQUIRED_PACKAGES}
    pronto_vis)
target_link_libraries (se-quick-lock lcm pronto_math
                        boost_system boost_thread)
pods_install_executables(se-quick-lock )