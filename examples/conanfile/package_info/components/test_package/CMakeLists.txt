cmake_minimum_required(VERSION 3.15)
project(PackageTest CXX)

find_package(game-engine CONFIG REQUIRED)

add_executable(example src/example.cpp)

target_link_libraries(example game-engine::algorithms 
                              game-engine::network 
                              game-engine::ai 
                              game-engine::rendering)
