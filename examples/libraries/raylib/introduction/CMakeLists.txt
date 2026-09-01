cmake_minimum_required(VERSION 3.23)
project(runner_game)

find_package(raylib)

add_executable(runner_game main.cpp)
target_link_libraries(runner_game raylib)

if(EMSCRIPTEN)
    # Set the executable suffix to .html in order to generate a html page by
    # Emscripten (there is no way of setting this from a user toolchain or
    # conanfile as it is later overridden by the Emscripten toolchain)
    set(CMAKE_EXECUTABLE_SUFFIX ".html")
endif()
