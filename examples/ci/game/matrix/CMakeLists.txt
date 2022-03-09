cmake_minimum_required(VERSION 3.15)
project(matrix CXX)

add_library(matrix src/matrix.cpp)
target_include_directories(matrix PUBLIC include)

set_target_properties(matrix PROPERTIES PUBLIC_HEADER "include/matrix.h")
install(TARGETS matrix)
