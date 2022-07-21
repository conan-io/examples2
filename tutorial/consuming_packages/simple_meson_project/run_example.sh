#!/bin/bash

set -e
set -x

BASEDIR=$(dirname "$0")
pushd "$BASEDIR"

rm -rf build
rm -rf src/meson

conan install . --output-folder=build --build=missing
meson setup --native-file build/conan_meson_native.ini . src/meson
meson compile -C src/meson
./src/meson/compressor
