set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

add_executable(se-fusion fusion.cpp)
pods_use_pkg_config_packages(se-fusion
    mav-state-est
#    mav-est-fovis
    gpf-laser-lib
    mav-est-legodo
    mav-est-legodo-external
    lcmtypes_pronto
    model-client 
    pronto_vis
    mav-est-pose-init)

# don't know why this is necessary (something to do with concise args):
target_link_libraries(se-fusion
    boost_system)
pods_install_executables(se-fusion)
