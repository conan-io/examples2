cmake_minimum_required(VERSION 3.15)
project(ai CXX)

find_package(mathlib CONFIG REQUIRED)

add_library(ai src/ai.cpp)
target_include_directories(ai PUBLIC include)
target_link_libraries(ai mathlib::mathlib)

set_target_properties(ai PROPERTIES PUBLIC_HEADER "include/ai.h")
install(TARGETS ai)
