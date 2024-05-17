#!/bin/bash
export CC=`command -v gcc-7`
export CXX=`command -v g++-7`
catkin_make -DCMAKE_BUILD_TYPE=Release --cmake-args -DCMAKE_C_COMPILER="$CC" -DCMAKE_CXX_COMPILER="$CXX"
