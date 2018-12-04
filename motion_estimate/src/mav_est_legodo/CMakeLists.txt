# added fro joint_utils vector assignment:
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

file(GLOB hpp_files *.hpp)

set(REQUIRED_LIBS mav-state-est 
    bot2-core 	bot2-param-client
    bot2-frames	eigen-utils
    path-util   )

# make the headers public
pods_install_headers(${hpp_files} DESTINATION mav_state_est/mav-est-legodo)

# make library that computes leg odo internally
add_library(mav-est-legodo SHARED 
                rbis_legodo_update.cpp
                rbis_legodo_common.cpp)

pods_use_pkg_config_packages(mav-est-legodo ${REQUIRED_LIBS}  #pronto_vis
    leg_estimate torque_adjustment_tools)

pods_install_libraries(mav-est-legodo)
pods_install_pkg_config_file(mav-est-legodo
    CFLAGS
    LIBS -lmav-est-legodo 
    REQUIRES 	${REQUIRED_LIBS} 
    VERSION 0.0.1)
