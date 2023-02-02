@ECHO ON

RMDIR /Q /S say\build
RMDIR /Q /S hello\build

REM Put say package in editable mode, and build it
conan editable add say/ say/0.1@user/channel

PUSHD "say"
REM It is very important to install 2 configurations, if you are building later the 2 configs, so the toolchain is multi-config (runtime libs)
REM There is a bug in 1.40, this needs to be done in the root folder
conan install . -s build_type=Release
conan install . -s build_type=Debug
MKDIR "build"
PUSHD "build"
cmake .. -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake
cmake --build . --config Release
cmake --build . --config Debug
POPD
POPD

REM Build hello consumer
MKDIR "hello/build"
PUSHD "hello/build"
conan install .. -s build_type=Release
conan install .. -s build_type=Debug
cmake .. -DCMAKE_TOOLCHAIN_FILE=generators/conan_toolchain.cmake
cmake --build . --config Release
cmake --build . --config Debug
"Release/hello.exe"
"Debug/hello.exe"
POPD

REM Do a modification in the source code of the editable say
PUSHD "say/build"
COPY ..\src\say2.cpp ..\src\say.cpp /Y
TOUCH ..\src\say.cpp
cmake --build . --config Release
cmake --build . --config Debug
POPD

REM Build and execute the consumer depending on the editable
PUSHD "hello/build"
cmake --build . --config Release
cmake --build . --config Debug
"Release/hello.exe"
"Debug/hello.exe"
POPD


conan editable remove say/0.1@user/channel

# Restore the say.cpp file to keep the repo unchanged
COPY say\src\original_say.cpp say\src\say.cpp /Y
