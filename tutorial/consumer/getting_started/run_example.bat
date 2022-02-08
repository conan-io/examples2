@ECHO ON

set BASEDIR=%~dp0
PUSHD %BASEDIR%

conan install . --output-folder cmake-build
cmake . -G "Visual Studio 15" -DCMAKE_TOOLCHAIN_FILE=conan_toolchain.cmake
cmake --build . --config Release
compressor.exe
