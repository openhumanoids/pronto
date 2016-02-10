add_definitions(-std=gnu99)
# Create a shared library with a single source file
add_library(bot2-frames-renderers-pronto SHARED
    coord_frames_renderer.c )

set(REQUIRED_LIBS bot2-core bot2-param-client bot2-vis bot2-frames)
  
pods_use_pkg_config_packages(bot2-frames-renderers-pronto ${REQUIRED_LIBS})

target_link_libraries(bot2-frames-renderers-pronto )

# set the library API version.  Increment this every time the public API
# changes.
set_target_properties(bot2-frames-renderers-pronto PROPERTIES SOVERSION 1)

# make the header public
# install it to include/bot_frames
pods_install_headers(bot_frames_renderers.h DESTINATION bot_frames_pronto)

# make the library public
pods_install_libraries(bot2-frames-renderers-pronto)

# create a pkg-config file for the library, to make it for other software to
# use it.
pods_install_pkg_config_file(bot2-frames-renderers-pronto
    CFLAGS
    LIBS -lbot2-frames-renderers-pronto
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)
