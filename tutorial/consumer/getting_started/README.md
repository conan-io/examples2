conan install . --output-folder conan-release

cmake . -DCMAKE_TOOLCHAIN_FILE=conan-release/conan_toolchain.cmake

cmake --build .

./compressor