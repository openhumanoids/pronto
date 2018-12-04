set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(yawlock)


set(REQUIRED_LIBS mav-state-est  bot2-core   bot2-param-client
    bot2-frames eigen-utils
    path-util   )


# make the headers public
pods_install_headers(rbis_yawlock_update.hpp yawlock.hpp DESTINATION mav_state_est/mav-est-yawlock)

# make library that computes leg odo internally
add_library(mav-est-yawlock SHARED 
                rbis_yawlock_update.cpp yawlock.cpp)

pods_use_pkg_config_packages(mav-est-yawlock ${REQUIRED_LIBS}  #pronto_vis
    )

pods_install_libraries(mav-est-yawlock)
pods_install_pkg_config_file(mav-est-yawlock
    CFLAGS
    LIBS -lmav-est-yawlock 
    REQUIRES  ${REQUIRED_LIBS} 
    VERSION 0.0.1)



################
add_executable(se-yawlock yawlock_app.cpp)
pods_use_pkg_config_packages(se-yawlock mav-est-yawlock
    model-client   bot2-param-client  urdf orocos-kdl kdl-parser forward-kinematics
    path-util ${REQUIRED_PACKAGES}
    ) #pronto_vis)
target_link_libraries (se-yawlock lcm pronto_math
                        boost_system boost_thread)
pods_install_executables(se-yawlock )


################ quick lock isn't used anymore #####
add_executable(se-quick-lock quick_lock.cpp)
pods_use_pkg_config_packages(se-quick-lock
    path-util ${REQUIRED_PACKAGES}
    ) #pronto_vis)
target_link_libraries (se-quick-lock lcm pronto_math
                        boost_system boost_thread)
pods_install_executables(se-quick-lock )
