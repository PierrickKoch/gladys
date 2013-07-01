cmake_minimum_required(VERSION 2.6)

add_library( gladys SHARED gladys.cpp nav_graph.cpp )
target_link_libraries( gladys ${GDAL_LIBRARIES} )
