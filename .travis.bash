#!/bin/bash
echo "GLADYS"
echo "======"
uname -a # show kernel info
n=`cat /proc/cpuinfo | grep "cpu cores" | uniq | awk '{print $NF}'`

echo "Download CMake GDAL and Boost (build w/$n cores)"

mkdir devel
export DEVEL_ROOT=$(pwd)/devel
#export BOOST_ROOT=$DEVEL_ROOT # used by FindBoost.cmake
#(wget -q http://sf.net/projects/boost/files/boost/1.54.0/boost_1_54_0.tar.bz2
#tar jxf boost_1_54_0.tar.bz2 && cd boost_1_54_0 && ./bootstrap.sh --prefix=$BOOST_ROOT
#./b2 -j$n install )& boostpid=$!

sudo apt-get -q update
sudo apt-get -qy install cmake libgdal1-dev libboost-all-dev
#wait $boostpid && grep "VERSION" $BOOST_ROOT/include/boost/version.hpp

echo "========================================"
echo "Build, test and install"

set -e # exit on error
mkdir build && cd build
cmake -DCMAKE_INSTALL_PREFIX=$DEVEL_ROOT ..
make -j$n
make test
make install

echo "========================================"

