cmake_minimum_required(VERSION 2.6)

set(PACKAGE_NAME gladys)
set(PACKAGE_VERSION "0.1")

# C++11 for GCC 4.6
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

find_package(GDAL)
find_package(Boost COMPONENTS graph unit_test_framework REQUIRED)

include_directories(include)
include_directories( ${GDAL_INCLUDE_DIR} )

# Library and binary
add_subdirectory(src)
##

# Testing
include( CTest )

add_subdirectory(test)
##
