file(GLOB gladys_SRCS "*.cpp")
add_library( gladys SHARED ${gladys_SRCS} )
target_link_libraries( gladys ${GDALWRAP_LIBRARIES} )
install(TARGETS gladys DESTINATION ${CMAKE_INSTALL_LIBDIR})
install_pkg_config_file(gladys
    DESCRIPTION "Graph Library for Autonomous and Dynamic Systems"
    CFLAGS
    LIBS -lgladys
    VERSION ${PACKAGE_VERSION})

add_executable( region_graph main/region_graph.cpp )
target_link_libraries( region_graph gladys )
install(TARGETS region_graph DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable( region_weight main/region_weight.cpp )
target_link_libraries( region_weight gladys )
install(TARGETS region_weight DESTINATION ${CMAKE_INSTALL_BINDIR})
