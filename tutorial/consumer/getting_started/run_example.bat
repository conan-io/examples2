@ECHO ON

set BASEDIR=%~dp0
PUSHD %BASEDIR%

conan install . --output-folder cmake-build --build=missing
cmake . -G "Visual Studio 15 2017" -DCMAKE_TOOLCHAIN_FILE=cmake-build/conan_toolchain.cmake
cmake --build . --config Release
compressor.exe
