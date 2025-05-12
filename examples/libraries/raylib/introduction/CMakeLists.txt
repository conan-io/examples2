cmake_minimum_required(VERSION 3.23)
project(runner_game)

set(CMAKE_CXX_STANDARD 17)
find_package(raylib)

add_executable(runner_game main.cpp)
target_link_libraries(runner_game raylib)
