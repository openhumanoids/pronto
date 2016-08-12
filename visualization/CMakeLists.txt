cmake_minimum_required(VERSION 2.6.0)

file(GLOB hpp_files include/visualization/*.hpp)
#add_definitions(-Wall -msse2 -msse3)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME visualization)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

find_package(Boost COMPONENTS thread system REQUIRED)
message(STATUS ${Boost_LIBRARIES})

find_package(GLib2 REQUIRED COMPONENTS gthread)
message(STATUS $(GLib2_LIBRARIES))

include_directories(include)
include_directories(${LCMTYPES_INCLUDE_DIRS})
link_libraries(${LCMTYPES_LIBS} vs_vis)

add_library(vs_vis SHARED
	src/collections_renderer/viewer.cpp
    )    
target_link_libraries(vs_vis ${Boost_LIBRARIES} GLib2::glib GLib2::gthread)

set_target_properties(vs_vis PROPERTIES SOVERSION 1)
pods_use_pkg_config_packages(vs_vis bot2-vis bot2-lcmgl-renderer lcm eigen3)

pods_install_libraries(vs_vis)
pods_install_headers(${hpp_files} DESTINATION visualization)

pods_install_pkg_config_file(visualization
    LIBS -l${LCMTYPES_LIBS} -lvs_vis -lglut -lgthread-2.0
    REQUIRES lcm 
    VERSION 0.0.1)

# individual collections renderer:
add_subdirectory(src/collections_renderer)
# viewer for just the above
add_subdirectory(src/collections_viewer)



