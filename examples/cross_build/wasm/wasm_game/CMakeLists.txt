cmake_minimum_required(VERSION 3.23)
project(wasm_game)

set(CMAKE_CXX_STANDARD 17)
find_package(raylib)

add_executable(wasm_game main.cpp)
target_link_libraries(wasm_game raylib)

set(CMAKE_EXECUTABLE_SUFFIX ".html")

set_target_properties(wasm_game PROPERTIES
    LINK_FLAGS "-sALLOW_MEMORY_GROWTH=1 -sUSE_GLFW=3 -sASYNCIFY --shell-file ${CMAKE_SOURCE_DIR}/shell.html"
)
