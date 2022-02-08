conan install .
cmake . -DCMAKE_TOOLCHAIN_FILE=cmake-build-release/conan/conan_toolchain.cmake
cmake --build .
./compressor
