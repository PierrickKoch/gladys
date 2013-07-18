cmake_minimum_required(VERSION 2.8)

set(PACKAGE_NAME gladys)
set(PACKAGE_VERSION "0.1")

# C++11 for GCC 4.6
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# Find GDAL ( export GDAL_ROOT=$prefix )
find_package(GDAL REQUIRED)

# Find Boost ( export BOOST_ROOT=$prefix )
set(BOOST_MIN_VERSION "1.46.0") # 1.47 for Boost.Geometry (Ubuntu > 12.04)
find_package(Boost ${BOOST_MIN_VERSION} COMPONENTS graph unit_test_framework REQUIRED)

include_directories(include)
include_directories(${GDAL_INCLUDE_DIR})
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# Filesystem Hierarchy Standard
include(GNUInstallDirs)

# Load the 'install_pkg_config_file' macro
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")
include(InstallPkgConfigFile)

# Library and binary
add_subdirectory(src)
##

# Testing
include( CTest )
add_subdirectory(test)
##

# Install headers
file(GLOB gladys_HDRS "include/gladys/*.hpp")
install(FILES ${gladys_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gladys)
##
