#!/bin/bash

set -e
set -x

BASEDIR=$(dirname "$0")
pushd "$BASEDIR"

rm -rf cmake-build-release
rm -rf cmake-build-debug

conan install . --output-folder cmake-build-release --settings build_type=Release --build=missing
cd cmake-build-release
cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake
cmake --build .
./compressor

cd ..

conan install . --output-folder cmake-build-debug --settings build_type=Debug --build=missing
cd cmake-build-debug
cmake .. -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake
cmake --build .
./compressor
