@ECHO ON

RMDIR /Q /S cmake-build-release

conan install . --output-folder cmake-build
cmake . -G "Visual Studio 15" -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake
cmake --build . --config Release
compressor.exe
