cmake_minimum_required(VERSION 3.15)
project(physx CXX)

find_package(matrix CONFIG REQUIRED)

add_library(physx src/physx.cpp)
target_include_directories(physx PUBLIC include)
target_link_libraries(physx matrix::matrix)

set_target_properties(physx PROPERTIES PUBLIC_HEADER "include/physx.h")
install(TARGETS physx)
