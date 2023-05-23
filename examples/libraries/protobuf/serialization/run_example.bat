@ECHO ON

RMDIR /Q /S build
MKDIR build
PUSHD build

pip install -r ../requirements.txt

conan install .. -pr:h=default -pr:b=default --build=missing
cmake .. -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake
cmake --build . --config Release

sensor.exe

SET PYTHONPATH="%CD%"
python ../main.py
