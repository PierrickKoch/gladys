cmake_minimum_required(VERSION 2.6)

set(PACKAGE_NAME gladys)
set(PACKAGE_VERSION "0.1")

# C++11 for GCC 4.6
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(GDAL)
find_package(Boost)

include_directories(include)
include_directories( ${Boost_INCLUDE_DIR} )
include_directories( ${GDAL_INCLUDE_DIR} )

add_executable( test_gladys src/gladys.cpp src/nav_graph.cpp )

target_link_libraries( test_gladys ${Boost_LIBRARIES} )
target_link_libraries( test_gladys ${GDAL_LIBRARIES} )

