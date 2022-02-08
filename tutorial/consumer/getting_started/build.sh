#!/bin/bash

set -e
set -x

rm -rf cmake-build-release

conan install . --output-folder cmake-build-release
cmake . -DCMAKE_TOOLCHAIN_FILE=cmake-build-release/conan_toolchain.cmake
cmake --build .
./compressor