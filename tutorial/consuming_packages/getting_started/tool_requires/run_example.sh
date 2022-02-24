#!/bin/bash

set -e
set -x

BASEDIR=$(dirname "$0")
pushd "$BASEDIR"

rm -rf cmake-build-release

conan install . --output-folder cmake-build-release --build=missing
source ./cmake-build-release/conanbuild.sh
cmake . -DCMAKE_TOOLCHAIN_FILE=conan/conan_toolchain.cmake
cmake --build .
source ./cmake-build-release/deactivate_conanbuild.sh
./compressor
