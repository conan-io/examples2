#!/bin/bash

set -e
set -x

rm -rf build/ CMakeUserPresets.json
mkdir build
pushd build

pip install -r ../requirements.txt

conan install .. -pr:h=default -pr:b=default --build=missing
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=Release/generators/conan_toolchain.cmake
cmake --build .

./sensor

PYTHONPATH=${PWD} python ../main.py
