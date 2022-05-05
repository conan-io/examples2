#!/bin/bash

set -e
set -x

BASEDIR=$(pwd)
pushd "$BASEDIR"

rm -rf tmp && mkdir tmp && cd tmp
conan new -d name=foo -d version=1.0 cmake_exe
conan install .
conan install . -s build_type=Debug
cmake --preset Debug
cmake --build --preset Debug
cmake --preset Release
cmake --build --preset Release


cd $BASEDIR
rm -rf tmp

