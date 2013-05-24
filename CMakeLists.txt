cmake_minimum_required(VERSION 2.6)

set(PACKAGE_NAME gladys)
set(PACKAGE_VERSION "0.1")

find_package(Boost)
find_package(GDAL)

INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )
INCLUDE_DIRECTORIES( ${GDAL_INCLUDE_DIR} )

ADD_EXECUTABLE( test src/gladys.cpp )

TARGET_LINK_LIBRARIES( test ${Boost_LIBRARIES} )
TARGET_LINK_LIBRARIES( test ${GDAL_LIBRARIES} )

