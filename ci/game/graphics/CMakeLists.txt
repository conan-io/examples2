cmake_minimum_required(VERSION 3.15)
project(graphics CXX)

find_package(mathlib CONFIG REQUIRED)

add_library(graphics src/graphics.cpp)
target_include_directories(graphics PUBLIC include)
target_link_libraries(graphics mathlib::mathlib)

set_target_properties(graphics PROPERTIES PUBLIC_HEADER "include/graphics.h")
install(TARGETS graphics)
