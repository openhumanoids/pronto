find_package(PkgConfig REQUIRED)
pkg_check_modules(BOT2_VIS bot2-vis)
if(NOT BOT2_VIS_FOUND)
    message("bot2-vis not found.  Not building libbot2 renderer")
    return()
endif(NOT BOT2_VIS_FOUND)

#add_definitions(-Wall -std=gnu99)

link_libraries(boost_thread)

add_executable(pronto_viewer pronto-viewer.cpp)
pods_use_pkg_config_packages(pronto-viewer bot2-vis 
  bot2-frames-renderers
  octomap-renderer
  occ-map-renderers
  laser-util-renderer
  bot2-lcmgl-renderer
  lcm
  collections_renderer
  mav-state-est-renderers
  path-util)


#        bot2-frames-renderers-drc  

pods_install_executables(pronto-viewer)




