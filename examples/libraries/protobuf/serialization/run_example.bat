@ECHO ON

RMDIR /Q /S build
MKDIR build
PUSHD build

CALL pip install -r ../requirements.txt

CALL conan install .. -pr:h=default -pr:b=default --build=missing
CALL cmake .. -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake
CALL cmake --build . --config Release

CALL Release\sensor.exe

SET PYTHONPATH=.
CALL python ../main.py
