@ECHO ON

set BASEDIR=%~dp0
PUSHD %BASEDIR%

RMDIR /Q /S cmake-build

conan install . --output-folderbuild --build=missing
cd cmake-build
cmake .. -G "Visual Studio 15 2017" -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake
cmake --build . --config Release
Release\compressor.exe
