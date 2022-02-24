@ECHO ON

set BASEDIR=%~dp0
PUSHD %BASEDIR%

RMDIR /Q /S cmake-build

conan install . --output-folder cmake-build --build=missing
./cmake-build/conanbuild.bat
cmake . -G "Visual Studio 15 2017" -DCMAKE_TOOLCHAIN_FILE=cmake-build/conan_toolchain.cmake
cmake --build . --config Release
./cmake-build/deactivate_conanbuild.bat
Release\compressor.exe
