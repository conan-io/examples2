cmake_minimum_required(VERSION 3.15)
project(foo CXX)

add_library(foo src/foo.cpp)
target_include_directories(foo PUBLIC include)

set_target_properties(foo PROPERTIES PUBLIC_HEADER "include/foo.h")
install(TARGETS foo)
