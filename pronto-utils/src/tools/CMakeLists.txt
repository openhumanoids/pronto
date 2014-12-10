set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable ")

add_executable(pronto-multisense-to-frame pronto_multisense_to_frame.cpp)
target_link_libraries(pronto-multisense-to-frame boost_system)
pods_use_pkg_config_packages(pronto-multisense-to-frame lcm 
  pronto_vis pronto_lcm)
pods_install_executables(pronto-multisense-to-frame)
