add_library( gladys SHARED gladys.cpp nav_graph.cpp )
target_link_libraries( gladys ${GDAL_LIBRARIES} )
install(TARGETS gladys DESTINATION lib)
install_pkg_config_file(gladys
    DESCRIPTION Graph Library for Autonomous and Dynamic Systems
    CFLAGS
    LIBS -lgladys
    REQUIRES GDAL
    VERSION ${PACKAGE_VERSION})

add_executable( region_graph region_graph.cpp )
target_link_libraries( region_graph gladys )
install(TARGETS region_graph DESTINATION bin)

add_executable( region_weight region_weight.cpp )
target_link_libraries( region_weight gladys )
install(TARGETS region_weight DESTINATION bin)
