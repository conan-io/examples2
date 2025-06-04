cmake_minimum_required(VERSION 3.23)
project(wasm_game)

set(CMAKE_CXX_STANDARD 17)
find_package(raylib)

add_executable(wasm_game main.cpp)
target_link_libraries(wasm_game raylib)

# Set the executable suffix to .html in order to generate a html page by
# Emscripten (there is no way of setting this from a user toolchain or
# conanfile as it is later overridden by the Emscripten toolchain)
set(CMAKE_EXECUTABLE_SUFFIX ".html")
