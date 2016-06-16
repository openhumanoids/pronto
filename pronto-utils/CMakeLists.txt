set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable")

cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME pronto-tools)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
#include(cmake/lcmtypes.cmake)
#lcmtypes_build()
#lcmtypes_build(C_AGGREGATE_HEADER pronto_tools.h)

add_subdirectory(src/pronto_math)
add_subdirectory(src/conversions)
add_subdirectory(src/pronto_vis)
add_subdirectory(src/pronto_lcm)
add_subdirectory(src/pronto_filters)
add_subdirectory(src/pronto_joint_tools)


add_subdirectory(src/test)
add_subdirectory(src/tools)

