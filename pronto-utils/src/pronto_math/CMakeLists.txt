add_definitions(-Wall -fPIC -enable-shared  -enable-pic)
add_library(pronto_math  SHARED pronto_math.cpp)

target_link_libraries(pronto_math jpeg ${LCMTYPES_LIBS})
pods_use_pkg_config_packages(pronto_math
    eigen3
    )
#  lcm eigen3   bot2-core) pcl_common glib-2.0)

set_target_properties(pronto_math PROPERTIES SOVERSION 1)
pods_install_libraries(pronto_math)
pods_install_headers(pronto_math.hpp DESTINATION pronto_utils)
pods_install_pkg_config_file(pronto_math
    LIBS -lpronto_math
    VERSION 0.0.1)

#    REQUIRES bot2-vis lcm visualization lcmtypes_kinect lcmtypes_openni_utils pcl_io cminpack

# removed: lcmtypes_openni-lcm
