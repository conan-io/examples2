@ECHO ON

set BASEDIR=%~dp0
PUSHD %BASEDIR%

RMDIR /Q /S build
RMDIR /Q /S src\meson

conan install . --output-folder=build --build=missing
meson setup --native-file build\conan_meson_native.ini . src\meson
meson compile -C src\meson
src\meson\compressor.exe
