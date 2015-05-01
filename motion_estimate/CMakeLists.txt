cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME motion_estimate)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# dehann:
add_subdirectory(src/foot_contact)
# mfallon:
add_subdirectory(src/foot_contact_alt)

add_subdirectory(src/leg_estimate)

# RGBD as a GPF
##add_subdirectory(src/gpf-rgbd-lib)

# mav modules :
#add_subdirectory(src/mav_est_fovis)
add_subdirectory(src/mav_est_legodo)
add_subdirectory(src/pose_meas)

# estimator proper
add_subdirectory(src/fusion)

# map creator:
add_subdirectory(src/create_octomap)

# gui:
add_subdirectory(src/pronto_viewer)


add_subdirectory(src/quick_lock)



pods_install_python_script(se-bdi-imu-individually scripts/republish_bdi_imu_individually.py)
file(COPY scripts/se-batch-process.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
pods_install_python_script(se-footstep-spoof scripts/footstep_spoof.py)


pods_install_python_script(se-state-sync-simple scripts/state-sync-simple.py)

pods_install_python_script(se-simple-restart scripts/se-simple-restart.py)

pods_install_python_script(se-filter-atlas-state scripts/filterAtlasState.py)
