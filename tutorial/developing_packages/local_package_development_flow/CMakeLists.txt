cmake_minimum_required(VERSION 3.15)
project(say CXX)

add_library(say src/say.cpp)
target_include_directories(say PUBLIC include)

set_target_properties(say PROPERTIES PUBLIC_HEADER "include/say.h")
install(TARGETS say)
