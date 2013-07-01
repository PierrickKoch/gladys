cmake_minimum_required(VERSION 2.6)

project(gladys)
set(PACKAGE_NAME gladys)
set(PACKAGE_VERSION "0.1")

# C++11 for GCC 4.6
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(GDAL)
find_package(Boost COMPONENTS unit_test_framework REQUIRED)

include_directories(include)
include_directories( ${Boost_INCLUDE_DIR} )
include_directories( ${GDAL_INCLUDE_DIR} )

link_directories( ${Boost_LIBRARY_DIRS} )

# Library and binary
add_subdirectory(src)

# Testing
add_subdirectory(test)
