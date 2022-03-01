@ECHO ON

set BASEDIR=%~dp0
PUSHD %BASEDIR%

RMDIR /Q /S cmake-build

conan install . --output-folder cmake-build --build=missing --settings build_type=Release
conan install . --output-folder cmake-build --build=missing --settings build_type=Debug
cmake . -G "Visual Studio 15 2017" -DCMAKE_TOOLCHAIN_FILE=cmake-build/conan_toolchain.cmake
cmake --build . --config Release
Release\compressor.exe
cmake --build . --config Debug
Debug\compressor.exe
