cmake_minimum_required(VERSION 2.6.0)

add_definitions(-fpermissive)

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
add_subdirectory(src/mav_est_fovis)
add_subdirectory(src/mav_est_legodo)
add_subdirectory(src/pose_meas)

add_subdirectory(src/quick_lock)

# estimator proper
add_subdirectory(src/fusion)

# cloud accumulate:
add_subdirectory(src/cloud_accumulate)

# map creator:
add_subdirectory(src/create_octomap)

# gui:
add_subdirectory(src/renderer_rs)
add_subdirectory(src/renderer_frames_pronto)
add_subdirectory(src/pronto_viewer)


# require python
find_package(PythonInterp REQUIRED)

# install all python files in the python/ subdirectory
pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/python)

pods_install_python_script(botpy botpy.main)



pods_install_python_script(se-bdi-imu-individually scripts/republish_bdi_imu_individually.py)
#file(COPY scripts/se-batch-process.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
pods_install_python_script(se-footstep-spoof scripts/footstep_spoof.py)


pods_install_python_script(se-state-sync-simple scripts/state-sync-simple.py)

pods_install_python_script(se-republish-atlas-state-as-core scripts/republish_atlas_state.py)
pods_install_python_script(se-simple-restart scripts/se-simple-restart.py)
pods_install_python_script(se-filter-atlas-state scripts/filterAtlasState.py)



pods_install_python_script(se-state-sync-simple-hyq scripts/state-sync-simple-hyq.py)

pods_install_python_script(se-send-a-pose scripts/send_a_pose.py)

