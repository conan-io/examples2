cmake_minimum_required(VERSION 3.15)
project(PackageTest CXX)

find_package(game-engine REQUIRED COMPONENTS algorithms network ai rendering)

add_executable(example src/example.cpp)

target_link_libraries(example algorithms 
                              network 
                              ai 
                              rendering)
