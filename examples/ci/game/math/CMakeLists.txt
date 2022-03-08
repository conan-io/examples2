cmake_minimum_required(VERSION 3.15)
project(math CXX)

add_library(math src/math.cpp)
target_include_directories(math PUBLIC include)

set_target_properties(math PROPERTIES PUBLIC_HEADER "include/math.h")
install(TARGETS math)
