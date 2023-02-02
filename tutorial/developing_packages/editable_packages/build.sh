#!/bin/bash

set -e
set -x

rm -rf say/build/
rm -rf hello/build/


conan editable add say/ say/0.1@user/channel

pushd say
conan install .
mkdir -p build/Release
pushd build/Release
cmake ../.. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake
cmake --build .
popd
popd

mkdir -p hello/build/Release
pushd hello/build/Release
conan install ../..
cmake ../.. -DCMAKE_BUILD_TYPE=Release -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake
cmake --build .
./hello
popd

# Modification to code (Changes the message 'hello' with 'bye')
pushd say/build/Release
cp ../../src/say2.cpp ../../src/say.cpp
cmake --build .
popd

# build consumer again
pushd hello/build/Release
cmake --build .
# This should print 'bye' instead of 'hello'
./hello
popd

conan editable remove say/0.1@user/channel

# Restore the say.cpp file to keep the repo unchanged
cp say/src/original_say.cpp say/src/say.cpp
