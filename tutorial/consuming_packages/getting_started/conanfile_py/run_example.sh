#!/bin/bash

set -e
set -x

BASEDIR=$(dirname "$0")
pushd "$BASEDIR"

rm -rf cmake-build-release

conan install . --output-folder cmake-build-release --build=missing
cmake . -DCMAKE_TOOLCHAIN_FILE=cmake-build-release/conan_toolchain.cmake
cmake --build .
./compressor
