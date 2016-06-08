#
# NBNBNBNBNBNBNNBNBNBNBNBNBNBNBNNBNB
# CONVENTION IS FOR RENDERER FILES TO BE NAMED
# renderer_name.cpp and renderer_name.hpp
# NBNBNBNBNBNBNNBNBNBNBNBNBNBNBNNBNB
#

find_package(PkgConfig REQUIRED)
add_definitions( -std=gnu99)

add_library(renderer_rs SHARED renderer_rs.cpp
  RobotStateListener.cpp
  InteractableGlKinematicBody.cpp
  GlKinematicBody.cpp )
  
set_target_properties(renderer_rs PROPERTIES SOVERSION 1)

set(REQUIRED_PACKAGES_ROBOT_STATE lcm lcmtypes_pronto bot2-core bot2-vis bot2-frames urdf  kdl-parser forward-kinematics eigen3 ) 


pods_install_libraries(renderer_rs)
pods_install_headers(renderer_rs.hpp   RobotStateListener.hpp
  InteractableGlKinematicBody.hpp eigen_kdl_conversions.hpp
  GlKinematicBody.hpp DESTINATION renderer_rs)
  
pods_install_pkg_config_file(renderer_rs
    LIBS -lrenderer_rs
    REQUIRES bot2-core bot2-vis bot2-frames path-util ${REQUIRED_PACKAGES_ROBOT_STATE} 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_rs
  eigen3 ${REQUIRED_PACKAGES_ROBOT_STATE}
  bot2-core bot2-vis bot2-frames)





