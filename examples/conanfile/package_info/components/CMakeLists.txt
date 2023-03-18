cmake_minimum_required(VERSION 3.15)
project(game-engine CXX)

add_library(ai src/ai.cpp)
add_library(rendering src/rendering.cpp)
add_library(algorithms src/algorithms.cpp)
add_library(network src/network.cpp)

target_include_directories(ai PUBLIC include)
target_include_directories(rendering PUBLIC include)
target_include_directories(algorithms PUBLIC include)
target_include_directories(network PUBLIC include)

set_target_properties(ai PROPERTIES PUBLIC_HEADER "include/ai.h")
set_target_properties(rendering PROPERTIES PUBLIC_HEADER "include/rendering.h")
set_target_properties(algorithms PROPERTIES PUBLIC_HEADER "include/algorithms.h")
set_target_properties(network PROPERTIES PUBLIC_HEADER "include/network.h")

target_link_libraries(ai algorithms)
target_link_libraries(rendering algorithms)

install(TARGETS ai rendering algorithms network)
