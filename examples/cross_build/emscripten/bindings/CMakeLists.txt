cmake_minimum_required(VERSION 3.15)
project(wasm_example CXX)

find_package(Eigen3 REQUIRED)
find_package(ZLIB REQUIRED)
find_package(fmt REQUIRED)
add_executable(wasm_example main.cpp)

target_link_libraries(${PROJECT_NAME} PRIVATE Eigen3::Eigen ZLIB::ZLIB fmt::fmt)

# Set the executable suffix to .html in order to generate a html page by
# Emscripten (there is no way of setting this from a user toolchain or
# conanfile as it is later overridden by the Emscripten toolchain)
set(CMAKE_EXECUTABLE_SUFFIX ".html")
