add_library( gladys SHARED gladys.cpp nav_graph.cpp )
target_link_libraries( gladys ${GDAL_LIBRARIES} )

add_executable( region_graph region_graph.cpp )
target_link_libraries( region_graph gladys )

add_executable( region_weight region_weight.cpp )
target_link_libraries( region_weight gladys )
