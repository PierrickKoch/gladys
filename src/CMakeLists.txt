add_library( gladys SHARED gladys.cpp nav_graph.cpp weight_map.cpp )
target_link_libraries( gladys ${GDAL_LIBRARIES} )
install(TARGETS gladys DESTINATION ${CMAKE_INSTALL_LIBDIR})
install_pkg_config_file(gladys
    DESCRIPTION "Graph Library for Autonomous and Dynamic Systems"
    CFLAGS
    LIBS -lgladys
    REQUIRES GDAL
    VERSION ${PACKAGE_VERSION})

add_executable( region_graph region_graph.cpp )
target_link_libraries( region_graph gladys )
install(TARGETS region_graph DESTINATION ${CMAKE_INSTALL_BINDIR})

add_executable( region_weight region_weight.cpp )
target_link_libraries( region_weight gladys )
install(TARGETS region_weight DESTINATION ${CMAKE_INSTALL_BINDIR})

if (PNG_FOUND)
    add_executable( gdal_float_rgb gdal_float_rgb.cpp )
    target_link_libraries( gdal_float_rgb png )
    target_link_libraries( gdal_float_rgb gladys )
    install(TARGETS gdal_float_rgb DESTINATION ${CMAKE_INSTALL_BINDIR})
endif(PNG_FOUND)

add_library( gladys_python SHARED gladys_python.cpp )
target_link_libraries( gladys_python gladys )
target_link_libraries( gladys_python ${Boost_LIBRARIES} )
target_link_libraries( gladys_python ${PYTHON_LIBRARIES} )
target_link_libraries( gladys_python ${GDAL_LIBRARIES} )
