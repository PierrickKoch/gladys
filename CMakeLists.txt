cmake_minimum_required(VERSION 2.8)

set(PACKAGE_NAME gladys)
set(PACKAGE_VERSION "0.2.6")

option(BUILD_TESTS  "Build tests" ON)
option(BUILD_PYTHON "Build Python bindings" ON)

# C++11 for GCC 4.6
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMakeModules")

# C++11 GDAL wrapper
find_package(GdalWrap REQUIRED)

# Find Boost ( export BOOST_ROOT=$prefix )
set(BOOST_MIN_VERSION "1.46.0") # 1.47 for Boost.Geometry (Ubuntu > 12.04)
find_package(Boost ${BOOST_MIN_VERSION} COMPONENTS graph unit_test_framework REQUIRED)

include_directories(include)
include_directories(${GDALWRAP_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})

# Filesystem Hierarchy Standard
include(GNUInstallDirs)

# Load the 'install_pkg_config_file' macro
include(InstallPkgConfigFile)

# RPath management http://www.cmake.org/Wiki/CMake_RPATH_handling
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
	set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}")
endif("${isSystemDir}" STREQUAL "-1")


# Library and binary
add_subdirectory(src)

# Python bindings
if (BUILD_PYTHON)
    # Find Python (optional)
    find_package(PythonLibs)
    find_package(PythonInterp)

    if (PYTHONINTERP_FOUND AND PYTHONLIBS_FOUND)
        find_package(Boost ${BOOST_MIN_VERSION} COMPONENTS python REQUIRED)
        include_directories(${PYTHON_INCLUDE_DIRS})
        add_subdirectory(python)
    else()
        message("Hey! Install Python [if you wanna gladysplay]")
    endif(PYTHONINTERP_FOUND AND PYTHONLIBS_FOUND)
endif(BUILD_PYTHON)

# Testing
if (BUILD_TESTS)
  include( CTest )
  add_subdirectory(test)
endif(BUILD_TESTS)

# Install headers
file(GLOB gladys_HDRS "include/gladys/*.hpp")
install(FILES ${gladys_HDRS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/gladys)

#########################################################################
### Uninstall ###
#########################################################################
configure_file("${CMAKE_HOME_DIRECTORY}/CMakeModules/cmake_uninstall.cmake.in" "cmake_uninstall.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P "cmake_uninstall.cmake")
