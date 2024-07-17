cmake_minimum_required(VERSION 3.15)
project(mathlib CXX)

add_library(mathlib src/mathlib.cpp)
target_include_directories(mathlib PUBLIC include)

set_target_properties(mathlib PROPERTIES PUBLIC_HEADER "include/mathlib.h")
install(TARGETS mathlib)
