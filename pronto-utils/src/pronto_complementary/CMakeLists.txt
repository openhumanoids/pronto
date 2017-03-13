set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")


add_definitions(-Wall )
#add_definitions(-Wall -std=gnu99) # .... compiler warning


add_library(pronto_complementary SHARED 
    complementary.cpp )
pods_use_pkg_config_packages(pronto_complementary eigen3)

target_link_libraries(pronto_complementary)

set_target_properties(pronto_complementary PROPERTIES SOVERSION 1)
pods_install_libraries(pronto_complementary)
pods_install_headers(complementary.hpp
    DESTINATION pronto_complementary)
pods_install_pkg_config_file(pronto_complementary
    LIBS -lpronto_complementary
    REQUIRES 
    VERSION 0.0.1)
  

##############################################
add_executable (se-complementary-test complementary_test.cpp)
target_link_libraries(se-complementary-test boost_system)
pods_use_pkg_config_packages(se-complementary-test eigen3
  lcm bot2-core bot2-frames   bot2-frames_cpp
  lcmtypes_bot2-core  pronto_complementary
  pronto_math
  path-util)
pods_install_executables(se-complementary-test)
