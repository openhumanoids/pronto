set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

pods_install_headers(pronto_frame_check_tools.hpp DESTINATION pronto_utils)
