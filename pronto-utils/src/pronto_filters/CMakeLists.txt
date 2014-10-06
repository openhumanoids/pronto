find_package(PkgConfig REQUIRED)
pkg_check_modules(VISUALIZATION visualization)
if(NOT VISUALIZATION_FOUND)
  message("---- Visualization not found. Not building pronto_filters\n")
  return()
endif()


################################################################
add_library(pronto_filters SHARED filter_planes.cpp grow_cloud.cpp 
	filter_findfloor.cpp)
#    filter_light.cpp filter_person.cpp 
#    filter_cloudtolaser.cpp)




pods_use_pkg_config_packages(pronto_filters   eigen3  
	pcl_common-1.7   pcl_io-1.7      pcl_surface-1.7   pcl_segmentation-1.7 
	pcl_filters-1.7  pcl_surface-1.7 pcl_io-1.7 	   pcl_kdtree-1.7  pcl_search-1.7)

target_link_libraries(pronto_filters)
# kmcl_utils qhull ${LCMTYPES_LIBS} pcl_common pcl_octree)
pods_use_pkg_config_packages(pronto_filters lcm eigen3   glib-2.0 )
# kmcl_utils 

# set the library API version.  Increment this every time the public API changes.
set_target_properties(pronto_filters PROPERTIES SOVERSION 1)
pods_install_libraries(pronto_filters)
pods_install_headers(filter_planes.hpp grow_cloud.hpp  
    filter_findfloor.hpp filter_colorize.hpp
    DESTINATION pronto_tools)
# filter_light.hpp 
#    filter_person.hpp   filter_cloudtolaser.hpp

pods_install_pkg_config_file(pronto_filters
    LIBS -lpronto_filters
    REQUIRES bot2-vis lcm  lcmtypes_visualization pronto_math
    VERSION 0.0.1) 
# visualization 


