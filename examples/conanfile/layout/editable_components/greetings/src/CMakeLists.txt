
set(CMAKE_CXX_COMPILER_WORKS 1)
set(CMAKE_CXX_ABI_COMPILED 1)
cmake_minimum_required(VERSION 3.0)
project(greetings CXX)

add_library(hello hello.cpp)
target_include_directories(hello PRIVATE hello)
add_library(bye bye.cpp)
target_include_directories(bye PRIVATE bye)
