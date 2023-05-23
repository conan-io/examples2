if "%CMAKE_GENERATOR%"=="" (
    ECHO CMAKE_GENERATOR environment variable not defined. Please define the CMake generator in the CMAKE_GENERATOR environment variable.
) else (
    @ECHO ON

    RMDIR /Q /S build
    MKDIR build
    PUSHD build

    conan install .. -pr:h=default -pr:b=default --build=missing
    cmake .. -G "%CMAKE_GENERATOR%" -A "%CMAKE_GENERATOR_PLATFORM%" -DCMAKE_TOOLCHAIN_FILE==Release/generators/conan_toolchain.cmake
    cmake --build . --config Release

    sensor.exe

    SET PYTHONPATH="%CD%"
    python ../main.py
)
