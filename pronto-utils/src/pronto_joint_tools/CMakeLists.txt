set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

# install it to include/joint_utils
pods_install_headers(pronto_joint_tools.hpp DESTINATION pronto_utils)

pods_install_headers(pronto_frame_check_tools.hpp DESTINATION pronto_utils)
