# - Find python libraries
# This module finds if Python is installed and determines where the
# include files and libraries are. It also determines what the name of
# the library is. This code sets the following variables:
#
#  PYTHONLIBS_FOUND           - have the Python libs been found
#  PYTHON_LIBRARIES           - path to the python library
#  PYTHON_INCLUDE_DIRS        - path to where Python.h is found

#=============================================================================
# Copyright 2001-2009 Kitware, Inc.
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file Copyright.txt for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================
# (To distribute this file outside of CMake, substitute the full
#  License text for the above reference.)

# EDIT 2013-08-26 use PkgConfig to get PYTHON_LIBRARY and PYTHON_INCLUDE_DIR
message("Using custom FindPythonLibs CMakeModule")

find_package(PkgConfig)

pkg_check_modules(PC_PYTHON REQUIRED python)
set(PYTHON_DEFINITIONS ${PC_PYTHON_CFLAGS_OTHER})
find_path(PYTHON_INCLUDE_DIR Python.h
    HINTS ${PC_PYTHON_INCLUDEDIR} ${PC_PYTHON_INCLUDE_DIRS})
find_library(PYTHON_LIBRARY NAME python${PC_PYTHON_VERSION}
    HINTS ${PC_PYTHON_LIBDIR} ${PC_PYTHON_LIBRARY_DIRS} )
set(PYTHON_INCLUDE_DIRS ${PYTHON_INCLUDE_DIR})
set(PYTHON_LIBRARIES ${PYTHON_LIBRARY})

include(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(PythonLibs DEFAULT_MSG PYTHON_LIBRARIES PYTHON_INCLUDE_DIRS)
